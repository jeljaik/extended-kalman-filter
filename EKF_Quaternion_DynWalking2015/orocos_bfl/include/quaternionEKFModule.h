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

#ifndef __QUATERNIONEKFMODULE_H__
#define __QUATERNIONEKFMODULE_H__

#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/RFModule.h>
#include <quaternionEKFThread.h>
#define FILTER_GROUP_PARAMS_NAME "EKFPARAMS"

class quaternionEKFModule: public yarp::os::RFModule
{
    double period;
    std::string robotName;
    std::string local;
    bool autoconnect;
    std::string mode;
    bool usingxsens;
    bool verbose;
    yarp::os::BufferedPort<yarp::sig::Vector> gyroMeasPort;
    quaternionEKFThread* quatEKFThread;
    
    // Parser parameters
    dataDumperParser* m_parser;
    currentData       m_currentData;
    
public:
    quaternionEKFModule();
    
    bool   configure(yarp::os::ResourceFinder &rf);
    bool   close();
    double getPeriod(){ return period; }
    bool   updateModule();
};



#endif


