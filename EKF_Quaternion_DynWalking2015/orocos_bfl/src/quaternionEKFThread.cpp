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
      m_sysPdf( STATEDIM ),
      m_prior_mu_vec( STATEDIM ),
      m_waitingTime( 0.0 )
{

}

void quaternionEKFThread::run()
{
    // Get Input and measurement
    bool reading = true;
    yarp::sig::Vector* imu_measurement = m_gyroMeasPort->read(reading);
    // Extract linear acceleration in m/s^2
    yarp::sig::Vector imu_linAcc(3); 
    imu_linAcc = imu_measurement->subVector(3,5);
    // NOTE The raw angular speed read from the IMU is in deg/s. In this module we will transform
    // it to rad/s
    yarp::sig::Vector imu_angVel(3);
    imu_angVel = imu_measurement->subVector(6,8);
    MatrixWrapper::ColumnVector input(m_input_size);
    input(1) = PI/180*imu_angVel(0);
    input(2) = PI/180*imu_angVel(1);
    input(3) = PI/180*imu_angVel(2);
    cout << "VEL INPUT IS: " << input << endl;
    // Extract accelerometer. Read from port!
    MatrixWrapper::ColumnVector measurement(m_measurement_size);
    measurement(1) = imu_linAcc(0);
    measurement(2) = imu_linAcc(1);
    measurement(3) = imu_linAcc(2);
    cout << "ACC INPUT IS: " << measurement << endl;

    // Noise gaussian
    // System Noise Mean
    MatrixWrapper::ColumnVector sys_noise_mu(m_state_size);
    // TODO sys_noise_mu should not be left zero
    sys_noise_mu = 0.0;
    MatrixWrapper::Matrix Xi(m_state_size, 3);
    XiOperator(m_posterior_state, &Xi);
    cout << "sys_noise_mu" << sys_noise_mu << endl;
//     sys_noise_mu = static_cast<MatrixWrapper::ColumnVector>( Xi*(m_period/(1000.0*2.0)) );

    // System Noise Covariance
    // TODO For now let's leave this constant as something to be tuned. 
    // This covariance matrix however should be computed as done in the matlab
    // utility ekfukf/lti_disc.m through Matrix Fraction Decomposition.
    MatrixWrapper::SymmetricMatrix sys_noise_cov(m_state_size);
    sys_noise_cov = 0.0;
    sys_noise_cov(1,1) = sys_noise_cov(2,2) = sys_noise_cov(3,3) = sys_noise_cov(4,4) = m_sigma_system_noise;
    
    m_sysPdf.AdditiveNoiseMuSet(sys_noise_mu);
    m_sysPdf.AdditiveNoiseSigmaSet(sys_noise_cov);
    
    double elapsedTime = yarp::os::Time::now() - m_waitingTime;
    cout << "Elapsed time: " << elapsedTime << endl;
    
    if (elapsedTime > 10.0) {
        if(!m_filter->Update(m_sys_model, input, m_meas_model, measurement))
            yError(" [quaternionEKFThread::run] Update step of the Kalman Filter could not be performed\n");
    } else {
            if(!m_filter->Update(m_sys_model, input))
                yError(" [quaternionEKFThread::run] Update step of the Kalman Filter could not be performed\n");
    }
    
    // Get the posterior of the updated filter. Result of all the system model and meaurement information
    BFL::Pdf<BFL::ColumnVector> * posterior = m_filter->PostGet();
    // Posterior Expectation
    m_posterior_state = posterior->ExpectedValueGet();
    MatrixWrapper::Quaternion expectedValueQuat(m_posterior_state);
    // Posterior Covariance
    MatrixWrapper::SymmetricMatrix covariance(m_state_size);
    covariance = posterior->CovarianceGet();
    cout << "Posterior Mean: " << expectedValueQuat << endl;
    cout << "Posterior Covariance: " << posterior->CovarianceGet() << endl;
    cout << " " << endl;
    MatrixWrapper::ColumnVector eulerAngles(3);
    expectedValueQuat.getEulerAngles(string("xyz"), eulerAngles);
    cout << "Posterior Mean in Euler Angles (degrees): " << eulerAngles  << endl;
    
    // Publish results to port
    yarp::sig::Vector tmpVec(m_state_size);
    for (int i=1; i<m_posterior_state.size()+1; i++) {
        tmpVec(i-1) = m_posterior_state(i);
    }
    // Publish Euler Angles estimation to port
    yarp::sig::Vector tmpEuler(3);
    for (int i=1; i<eulerAngles.rows()+1; i++)
        tmpEuler(i-1) = eulerAngles(i)*(180/PI);
    yarp::sig::Vector& tmpPortEuler = m_publisherFilteredOrientationEulerPort->prepare();
    tmpPortEuler = tmpEuler;
    m_publisherFilteredOrientationEulerPort->write();
    yarp::sig::Vector& tmpPortRef = m_publisherFilteredOrientationPort->prepare();
    tmpPortRef = tmpVec;
    m_publisherFilteredOrientationPort->write();
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
    
    // Open publisher port for estimate in quaternion
    m_publisherFilteredOrientationPort = new yarp::os::BufferedPort<yarp::sig::Vector>;
    m_publisherFilteredOrientationPort->open(string("/" + m_moduleName + "/filteredOrientation:o").c_str());
    
    // Open publisher port for estimate in euler angles
    m_publisherFilteredOrientationEulerPort = new yarp::os::BufferedPort<yarp::sig::Vector>;
    m_publisherFilteredOrientationEulerPort->open(string("/" + m_moduleName + "/filteredOrientationEuler:o").c_str());
    
    // System Noise Mean
    MatrixWrapper::ColumnVector sys_noise_mu(m_state_size);
    sys_noise_mu(1) = sys_noise_mu(2) = sys_noise_mu(3) = sys_noise_mu(4) = 0;
    
    // System Noise Covariance
    MatrixWrapper::SymmetricMatrix sys_noise_cov(m_state_size);
    sys_noise_cov = 0.0;
    sys_noise_cov(1,1) = sys_noise_cov(2,2) = sys_noise_cov(3,3) = sys_noise_cov(4,4) = m_sigma_system_noise;
    
    // Setting System noise uncertainty
    m_sysPdf.AdditiveNoiseMuSet(sys_noise_mu);
    m_sysPdf.AdditiveNoiseSigmaSet(sys_noise_cov);
    m_sysPdf.setPeriod(m_period);
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
    // Setting prior. This is equivalent to a zero rotation
    MatrixWrapper::ColumnVector prior_mu(m_state_size);
    prior_mu = 0.0;
    prior_mu(1) = 1.0;
    m_prior_mu_vec = prior_mu;
    m_posterior_state = prior_mu;
    MatrixWrapper::SymmetricMatrix prior_cov(4);
    prior_cov = 0.0;
    prior_cov(1,1) = prior_cov(2,2) = prior_cov(3,3) = prior_cov(4,4) = m_prior_cov;
    cout << "Priors will be: " << endl;
    cout << "State prior: " << prior_mu << endl;
    cout << "Covariance prior: " << prior_cov << endl;
    m_prior = new BFL::Gaussian(prior_mu, prior_cov);
    
    // Construction of the filter
    m_filter = new BFL::ExtendedKalmanFilter(m_prior);
    
    // Sensor ports
    // This port was opened by the module.
    std::string gyroMeasPortName = string("/" + m_moduleName + "/imu:i");
    
    if (m_autoconnect) {
        yarp::os::ConstString src = std::string("/" + m_robotName + "/inertial");
        if(!yarp::os::Network::connect(src, gyroMeasPortName,"tcp")){
            yError(" [quaternionEKFThread::threadInit()] Connection with %s was not possible", gyroMeasPortName.c_str());
            return false;
        }
    }
    
    cout << "Thread waiting five seconds before starting..." <<  endl;
    yarp::os::Time::delay(5);
    
    
    m_waitingTime = yarp::os::Time::now();
    return true;
}

void quaternionEKFThread::XiOperator ( MatrixWrapper::ColumnVector quat, MatrixWrapper::Matrix* Xi )
{
//     In  Matlab language this would be:
//     Xi = [       -qk(2:4,:)'           ;
//           qk(1)*eye(3) + S(qk(2:4,:)) ];
    
    MatrixWrapper::ColumnVector omg(3);
    omg(1) = quat(2);    omg(2) = quat(3);    omg(3) = quat(4);
    
    (*Xi)(1,1) = -quat(2);    (*Xi)(1,2) = -quat(3);    (*Xi)(1,3) = -quat(4);
    MatrixWrapper::Matrix eye(3,3);
    eye.toIdentity();
    MatrixWrapper::Matrix S(3,3);
    SOperator(omg, &S);
    Xi->sub(2,4,1,3) = eye*quat(1) + S;
}

void quaternionEKFThread::SOperator ( MatrixWrapper::ColumnVector omg, MatrixWrapper::Matrix* S )
{
    (*S)(1,1) = 0.0;    (*S)(1,2) = -omg(3); (*S)(1,3) = omg(2);
    (*S)(2,1) = omg(3); (*S)(2,2) = 0.0    ; (*S)(2,3) = -omg(1);
    (*S)(3,1) = -omg(2);(*S)(3,2) = omg(1) ; (*S)(3,3) = 0.0;
}


void quaternionEKFThread::threadRelease()
{
    if (m_parser) { 
        delete m_parser;
        m_parser = NULL;
    }
    if (m_publisherFilteredOrientationEulerPort) {
        delete m_publisherFilteredOrientationEulerPort;
        m_publisherFilteredOrientationEulerPort = NULL;
    }
    if (m_publisherFilteredOrientationPort) {
        delete m_publisherFilteredOrientationPort;
        m_publisherFilteredOrientationPort = NULL;
    }
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

