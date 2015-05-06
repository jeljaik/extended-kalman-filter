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
    : RateThread ( m_period ),
      m_moduleName ( moduleName),
      m_robotName ( robotName),
      m_autoconnect ( autoconnect ),
      m_filterParams( filterParams ),
      m_gyroMeasPort ( gyroMeasPort ),
      m_sysPdf(STATEDIM)
{

}

void quaternionEKFThread::run()
{
    // Get Input and measurement
    // Read port 
    yarp::sig::Vector* imu_measurement = m_gyroMeasPort->read(false);
    // Extract angular velocity. Read from port!
    yarp::sig::Vector imu_linAcc = imu_measurement->subVector(3,5);
    yarp::sig::Vector imu_angVel = imu_measurement->subVector(6,8);
    MatrixWrapper::ColumnVector input(m_input_size);
    input(1) = imu_angVel(0);
    input(2) = imu_angVel(1);
    input(3) = imu_angVel(2);
    // Extract accelerometer. Read from port!
    MatrixWrapper::ColumnVector measurement(m_measurement_size);
    measurement(1) = imu_linAcc(0);
    measurement(2) = imu_linAcc(1);
    measurement(3) = imu_linAcc(2);
    
    // Perform new estimation
    m_filter->Update(m_sys_model, input, m_meas_model, measurement);
    // Get the posterior of the updated filter. Result of all the system model and meaurement information
    BFL::Pdf<BFL::ColumnVector> * posterior = m_filter->PostGet();
    cout << "Posterior Mean: " << posterior->ExpectedValueGet() << endl;
    cout << "Posterior Covariance: " << posterior->CovarianceGet() << endl;
    cout << " " << endl; 
}

bool quaternionEKFThread::threadInit()
{
    yarp::os::Bottle outputParamsBottle = m_filterParams.findGroup(FILTER_GROUP_PARAMS_NAME);
    if (!outputParamsBottle.isNull()) {
        m_state_size = outputParamsBottle.find("STATE_SIZE").asInt();
        m_input_size = outputParamsBottle.find("INPUT_SIZE").asInt();
        m_measurement_size = outputParamsBottle.find("MEASUREMENT_SIZE").asInt();
        m_prior_state_cov = outputParamsBottle.find("PRIOR_COV_STATE").asDouble();
        m_mu_system_noise = outputParamsBottle.find("MU_SYSTEM_NOISE").asDouble();
        m_sigma_system_noise = outputParamsBottle.find("SIGMA_SYSTEM_NOISE").asDouble();
        m_sigma_measurement_noise = outputParamsBottle.find("SIGMA_MEASUREMENT_NOISE").asDouble();
        m_prior_mu = outputParamsBottle.find("PRIOR_MU_STATE").asDouble();
        m_prior_cov = outputParamsBottle.find("PRIOR_COV_STATE").asDouble();
    } else {
        yError("Filter parameters from configuration file could not be extracted");
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
    meas_noise_mu(4) = GRAVITY_ACC;
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
    
    // Prior
    // TODO We need a zero method in MatrixWrapper for both vectors and matrices
    MatrixWrapper::ColumnVector prior_mu(m_state_size);
    prior_mu(1) = prior_mu(2) = prior_mu(3) = 0.0; 
    prior_mu(4) = m_prior_mu;
    MatrixWrapper::SymmetricMatrix prior_cov(m_state_size);
    // TODO Setting matrix to zero. This is specifically for BOOST
    for (unsigned int i = 1; i < prior_cov.size1() + 1; ++i)
        for (unsigned int j = 1; j < prior_cov.size2() + 1; ++j)
            prior_cov(i,j) = 0;
    prior_cov(1,1) = prior_cov(2,2) = prior_cov(3,3) = prior_cov(4,4) = m_prior_cov;
    m_prior = new BFL::Gaussian(m_prior_mu, m_prior_cov);
    
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

