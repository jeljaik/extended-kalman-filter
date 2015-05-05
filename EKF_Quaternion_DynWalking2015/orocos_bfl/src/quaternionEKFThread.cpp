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
#include <../../example/game/game_server/Matrix.h>

quaternionEKFThread::quaternionEKFThread ( int period, 
                                           yarp::os::Property &filterParams)
    : RateThread ( m_period ),
      m_filterParams( filterParams ),
      m_sysPdf(STATEDIM)
{
    

}

void quaternionEKFThread::run()
{
    
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
    
    // System noise uncertainty
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
    // Nonlinear measurement model
    MatrixWrapper::Matrix H(m_measurement_size, m_state_size);
    H = 0.0;
    return true;
}

void quaternionEKFThread::threadRelease()
{
    if ( m_parser ) { 
        delete m_parser;
        m_parser = NULL;
    }
    m_parser = NULL;
    if(m_sys_model) {
        delete m_sys_model;
        m_sys_model = NULL;
    }
}

