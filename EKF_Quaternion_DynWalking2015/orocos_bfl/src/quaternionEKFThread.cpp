/*
 * Copyright (C) 2014 Fondazione Istituto Italiano di Tecnologia - Italian Institute of Technology
 * Author: Jorhabib Eljaik
 * email:  jorhabib.eljaik@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include "quaternionEKFThread.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

quaternionEKFThread::quaternionEKFThread ( int period,
                                           std::string moduleName,
                                           std::string robotName,
                                           bool autoconnect,
                                           yarp::os::Property &filterParams,
                                           yarp::os::BufferedPort<yarp::sig::Vector>* gyroMeasPort
                                         )
    : RateThread ( period ),
      m_period ( period ),
      m_moduleName ( moduleName ),
      m_robotName ( robotName ),
      m_autoconnect ( autoconnect ),
      m_filterParams( filterParams ),
      m_gyroMeasPort ( gyroMeasPort ),
      m_sysPdf( STATEDIM )
{

}

void quaternionEKFThread::run()
{
    // Get Input and measurement
    // Read port 
    // When  using the simulator it's better to have a blocking reading as it's mainly done for debugging reasons and also because the simulator might not be running fast enough
    bool reading = false;
    if (!m_robotName.compare("icubGazeboSim"))
        reading = true;
    else
        reading = false;
    yarp::sig::Vector* imu_measurement = m_gyroMeasPort->read(reading);
    cout << "read imu_measurement was: " << imu_measurement->toString().c_str() << endl;
    // Extract angular velocity. Read from port!
    yarp::sig::Vector imu_linAcc(3); 
    imu_linAcc = imu_measurement->subVector(3,5);
    yarp::sig::Vector imu_angVel(3);
    imu_angVel = imu_measurement->subVector(6,8);
    MatrixWrapper::ColumnVector input(m_input_size);
    input(1) = imu_angVel(0);
    input(2) = imu_angVel(1);
    input(3) = imu_angVel(2);
    cout << "VEL INPUT IS: " << input << endl;
    // Extract accelerometer. Read from port!
    MatrixWrapper::ColumnVector measurement(m_measurement_size);
    measurement(1) = imu_linAcc(0);
    measurement(2) = imu_linAcc(1);
    measurement(3) = imu_linAcc(2);
    cout << "ACC INPUT IS: " << measurement << endl;
    
    // Time-varying linear system model
    // A = id(4) + 0.5*Omega(angVel)*period
    MatrixWrapper::Matrix A(m_state_size,m_state_size);
    // TODO This should be done in the Matrix wrapper
    boost::numeric::ublas::identity_matrix<double> id(m_state_size);
    MatrixWrapper::Matrix identity = MatrixWrapper::Matrix(id);
    // Omega operator
    MatrixWrapper::Matrix Omega(m_state_size, m_state_size);
    Omega(1,1) = 0.0;          Omega(1,2) = -input(1);   Omega(1,3) = -input(2);   Omega(1,4) = -input(3);
    Omega(2,1) = input(1);   Omega(2,2) =  0.0;          Omega(2,3) =  input(3);   Omega(2,4) = -input(2);
    Omega(3,1) = input(2);   Omega(3,2) = -input(3);   Omega(3,3) =  0.0;          Omega(3,4) =  input(1);
    Omega(4,1) = input(3);   Omega(4,3) =  input(2);   Omega(4,3) = -input(1);   Omega(4,4) =  0.0;
    MatrixWrapper::Matrix tmp = Omega*((double)(0.5*(m_period/1000.0)));
    A = static_cast<MatrixWrapper::Matrix>(identity) + static_cast<MatrixWrapper::Matrix>(Omega*(0.5*m_period/1000.0));
    cout << "Matrix A to be stored in AB: " << A << endl;
    // NOTE B must be of size 4 \times 3 to be consistent with Ax + Bu
    boost::numeric::ublas::zero_matrix<double> tmpB(m_state_size,m_input_size);
    MatrixWrapper::Matrix B(tmpB);
    B = 0.0;
    
    vector<MatrixWrapper::Matrix> AB(2);
    AB[0] = A;
    AB[1] = B;
    
    // Noise gaussian
    // System Noise Mean
    MatrixWrapper::ColumnVector sys_noise_mu(m_state_size);
    MatrixWrapper::ColumnVector Xi(static_cast<MatrixWrapper::ColumnVector>(boost::numeric::ublas::zero_vector<double>(m_state_size)));
    // TODO Do this in a smarter way PLEASE!!!
    cout << "sys_noise_mu" << sys_noise_mu << endl;
    sys_noise_mu = static_cast<MatrixWrapper::ColumnVector>( Xi*(m_period/(1000.0*2.0))*m_mu_gyro_noise );
//     sys_noise_mu(1) = sys_noise_mu(2) = sys_noise_mu(3) = sys_noise_mu(4) = m_mu_system_noise;
    
    // System Noise Covariance
    // TODO For now let's leave this constant as something to be tuned. 
    // This covariance matrix however should be computed as done in the matlab
    // utility ekfukf/lti_disc.m through Matrix Fraction Decomposition.
    MatrixWrapper::SymmetricMatrix sys_noise_cov(m_state_size);
    sys_noise_cov = 0.0;
    sys_noise_cov(1,1) = sys_noise_cov(2,2) = sys_noise_cov(3,3) = sys_noise_cov(4,4) = m_sigma_system_noise;
    
    // System model
    BFL::Gaussian system_uncertainty(sys_noise_mu, sys_noise_cov);
    cout << "Matrix A is: " << AB[0] << endl;
    cout << "Matrix B is: " << AB[1] << endl; 
    BFL::LinearAnalyticConditionalGaussian sys_pdf(AB, system_uncertainty);
    BFL::LinearAnalyticSystemModelGaussianUncertainty lin_sys_model(&sys_pdf);
    
    // Perform new estimation
    if(!m_filter->Update(&lin_sys_model, input, m_meas_model, measurement))
        yError(" [quaternionEKFThread::run] Update step of the Kalman Filter could not be performed\n");
    // Get the posterior of the updated filter. Result of all the system model and meaurement information
    BFL::Pdf<BFL::ColumnVector> * posterior = m_filter->PostGet();
    cout << "Posterior Mean: " << posterior->ExpectedValueGet() << endl;
    cout << "Posterior Covariance: " << posterior->CovarianceGet() << endl;
    cout << " " << endl; 
}

bool quaternionEKFThread::threadInit()
{
    if (!m_filterParams.isNull()) {
        m_state_size = m_filterParams.find("STATE_SIZE").asInt();
        m_input_size = m_filterParams.find("INPUT_SIZE").asInt();
        m_measurement_size = m_filterParams.find("MEASUREMENT_SIZE").asInt();
        m_prior_state_cov = m_filterParams.find("PRIOR_COV_STATE").asDouble();
        m_mu_system_noise = m_filterParams.find("MU_SYSTEM_NOISE").asDouble();
        m_sigma_system_noise = m_filterParams.find("SIGMA_SYSTEM_NOISE").asDouble();
        m_sigma_measurement_noise = m_filterParams.find("SIGMA_MEASUREMENT_NOISE").asDouble();
        m_prior_mu = m_filterParams.find("PRIOR_MU_STATE").asDouble();
        m_prior_cov = m_filterParams.find("PRIOR_COV_STATE").asDouble();
        m_mu_gyro_noise = m_filterParams.find("MU_GYRO_NOISE").asDouble();
    } else {
        yError(" [quaternionEKFThread::threadInit] Filter parameters from configuration file could not be extracted");
        return false;
    }
    
    // System Noise Mean
    MatrixWrapper::ColumnVector sys_noise_mu(m_state_size);
    sys_noise_mu(1) = sys_noise_mu(2) = sys_noise_mu(3) = sys_noise_mu(4) = m_mu_system_noise;
    
    // System Noise Covariance
    MatrixWrapper::SymmetricMatrix sys_noise_cov(m_state_size);
    sys_noise_cov = 0.0;
    sys_noise_cov(1,1) = sys_noise_cov(2,2) = sys_noise_cov(3,3) = sys_noise_cov(4,4) = m_sigma_system_noise;
    
    // Setting System noise uncertainty
    m_sysPdf.AdditiveNoiseMuSet(sys_noise_mu);
    m_sysPdf.AdditiveNoiseSigmaSet(sys_noise_cov);
    // Creating the model
    m_sys_model = new BFL::AnalyticSystemModelGaussianUncertainty(&m_sysPdf);
    
    // Creating measurement model for linear measurement model
    // Measurement noise distribution
    // Measurement noise mean
    MatrixWrapper::ColumnVector meas_noise_mu(m_measurement_size);
    // TODO Check that this is correct
    meas_noise_mu = 0.0;                // Set all to zero
    cout << "just a test: " << meas_noise_mu << endl;
    meas_noise_mu(3) = GRAVITY_ACC;
    // Measurement noise covariance
    MatrixWrapper::SymmetricMatrix meas_noise_cov(m_measurement_size);
    meas_noise_cov = 0.0;
    meas_noise_cov(1,1) = meas_noise_cov(2,2) = meas_noise_cov(3,3) = m_sigma_measurement_noise;
    // Measurement noise uncertainty
    m_measurement_uncertainty = new BFL::Gaussian(meas_noise_mu, meas_noise_cov);
    // Probability density function (PDF) for the measurement
    m_measPdf = new BFL::nonLinearMeasurementGaussianPdf(*m_measurement_uncertainty);
    //  Measurement model from the measurement PDF
    m_meas_model = new BFL::AnalyticMeasurementModelGaussianUncertainty(m_measPdf);
    // Setting prior
    MatrixWrapper::ColumnVector prior_mu(m_state_size);
    prior_mu = 0.0;
    prior_mu(4) = 0.01;
    cout << "m_prior_mu: " << m_prior_mu << endl;
    MatrixWrapper::SymmetricMatrix prior_cov(4);
    cout << "size of this matrix " << prior_cov.size() <<endl;
    prior_cov = 0.0;
    prior_cov(1,1) = prior_cov(2,2) = prior_cov(3,3) = prior_cov(4,4) = m_prior_cov;
    cout << "Priors will be: " << endl;
    cout << "m_prior_mu: " << m_prior_mu << endl;
    cout << "m_prior_cov: " << m_prior_cov << endl;
    m_prior = new BFL::Gaussian(prior_mu, prior_cov);
    
    // Construction of the filter
    m_filter = new BFL::ExtendedKalmanFilter(m_prior);
    
    // Sensor ports
    // This port was opened by the module.
    // TODO Pass this name to the thread!!!!
    std::string gyroMeasPortName = string("/" + m_moduleName + "/imu:i");
    
    if (m_autoconnect) {
        yarp::os::ConstString src = std::string("/" + m_robotName + "/inertial");
        if(!yarp::os::Network::connect(src, gyroMeasPortName,"tcp")){
            yError(" [quaternionEKFThread::threadInit()] Connection with %s was not possible", gyroMeasPortName.c_str());
            return false;
        }
    }
    return true;
}

void quaternionEKFThread::threadRelease()
{
    if (m_parser) { 
        delete m_parser;
        m_parser = NULL;
    }
    m_parser = NULL;
    if (m_sys_model) {
        delete m_sys_model;
        m_sys_model = NULL;
    }
    if (m_measurement_uncertainty) {
        delete m_measurement_uncertainty;
        m_measurement_uncertainty = NULL;
    }
    if (m_measPdf) {
        delete m_measPdf;
        m_measPdf = NULL;
    }
    if (m_meas_model) {
        delete m_meas_model;
        m_meas_model = NULL;
    }
    if (m_prior) { 
        delete m_prior;
        m_prior = NULL;
    }
    if (m_filter) {
        delete m_filter;
        m_filter = NULL;
    }
}

