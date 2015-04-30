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
    } else {
        yError("Filter parameters from configuration file could not be extracted");
        return false;
    }
    
    // System Noise Mean
    MatrixWrapper::ColumnVector sys_noise_mu(m_state_size);
    sys_noise_mu(1) = m_mu_system_noise;
    sys_noise_mu(2) = m_mu_system_noise;
    sys_noise_mu(3) = m_mu_system_noise;
    sys_noise_mu(4) = m_mu_system_noise;
    
    // System Noise Covariance
    MatrixWrapper::SymmetricMatrix sys_noise_cov(m_state_size);
    sys_noise_cov(1,2) = sys_noise_cov(1,3) = sys_noise_cov(1,4) = 0.0;
    sys_noise_cov(1,1) = m_sigma_system_noise;
    sys_noise_cov(2,1) = sys_noise_cov(2,3) = sys_noise_cov(2,4) = 0.0;
    sys_noise_cov(2,2) = m_sigma_system_noise;
    sys_noise_cov(3,1) = sys_noise_cov(3,2) = sys_noise_cov(3,4) = 0.0;
    sys_noise_cov(3,3) = m_sigma_system_noise;
    sys_noise_cov(4,1) = sys_noise_cov(4,2) = sys_noise_cov(4,3) = 0.0;
    sys_noise_cov(4,4) = m_sigma_system_noise;
    
//     BFL::Gaussian system_uncertainty(sys_noise_mu, sys_noise_cov);
    m_sysPdf.AdditiveNoiseMuSet(sys_noise_mu);
    m_sysPdf.AdditiveNoiseSigmaSet(sys_noise_cov);
    return true;
}

void quaternionEKFThread::threadRelease()
{
    delete m_parser;
    m_parser = NULL;
}

