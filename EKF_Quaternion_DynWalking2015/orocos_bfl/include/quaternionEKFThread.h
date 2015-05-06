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

#ifndef __QUATERNIONEKFTHREAD_H__
#define __QUATERNIONEKFTHREAD_H__

#include <bfl/filter/extendedkalmanfilter.h>
#include <bfl/model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <bfl/model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <bfl/pdf/analyticconditionalgaussian.h>
#include <bfl/pdf/linearanalyticconditionalgaussian.h>

#include <yarp/os/RateThread.h>
#include <yarp/os/Property.h>
#include <yarp/os/Bottle.h>

#include "nonLinearAnalyticConditionalGaussian.h"
#include "nonLinearMeasurementGaussianPdf.h"
#include "dataDumperParser.h"
//TODO The path to the original data file must be retrieved by the ResourceFinder.
#define DATAFILE "/home/jorhabib/Software/extended-kalman-filter/EKF_Quaternion_DynWalking2015/orocos_bfl/data/dumper/icub/inertial/data.log"
//TODO This should come from the configuration file
#define STATEDIM 4
//TODO In case you wanna add a different group in the configuration file
#define FILTER_GROUP_PARAMS_NAME "EKFPARAMS"
#define GRAVITY_ACC 9.81

class quaternionEKFThread: public yarp::os::RateThread
{
    int                                          m_period;
    yarp::os::Property                           m_filterParams;
    dataDumperParser*                            m_parser;
    // currentData struct defined in dataDumperParser.h
    currentData                                  m_currentData;
    BFL::nonLinearAnalyticConditionalGaussian    m_sysPdf;
    BFL::AnalyticSystemModelGaussianUncertainty* m_sys_model;
    BFL::Gaussian*                               m_measurement_uncertainty;
    BFL::nonLinearMeasurementGaussianPdf*        m_measPdf;
    BFL::AnalyticMeasurementModelGaussianUncertainty* m_meas_model;
    // filter parameters read from configuration file
    // TODO These should be put in some structure
    int m_state_size;
    int m_input_size;
    int m_measurement_size;
    double m_prior_state_cov;
    double m_mu_system_noise;
    double m_sigma_system_noise;
    double m_sigma_measurement_noise;
    // Priors
    BFL::Gaussian*   m_prior;
    double           m_prior_cov;
    double           m_prior_mu;
    // Filter
    BFL::ExtendedKalmanFilter*  m_filter;
    
public:
  quaternionEKFThread ( int period, yarp::os::Property &filterParams);
  bool threadInit();
  void run();
  void threadRelease();
};

#endif