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
#include <yarp/os/LogStream.h>
#include <yarp/os/Port.h>

#include "quaternionEKFThread.h"
#include "quaternionEKFModule.h"

quaternionEKFModule::quaternionEKFModule()
{
    period = 0.01;
}

bool quaternionEKFModule::configure ( yarp::os::ResourceFinder& rf )
{
    if( rf.check("robot") ) {
        robotName = rf.find("robot").asString();
    } else {
        yError("[quaternionEKFModule::configure] Configuration failed! No robot param was found");
        return false;
    }
    
    if( rf.check("rate") && rf.find("rate").asDouble() )
    {
        period = rf.find("rate").asDouble();
    } else {
        yError("[quaternionEKFModule::configure] Configuration failed. No rate was specified.");
        return false;
    }
    
    if( rf.check("local") ) {
        local =  rf.find("local").asString();
    } else {
        yError("[quaternionEKFModule::configure] Configuration failed. No local name was foundd.");
        return false;
    }
    
    if( rf.check("autoconnect") ) {
        autoconnect = rf.find("autoconnect").asBool();
    } else {
        yError("[quaternionEKFModule::configure] Configuration failed. No value for autoconnect was found.");
        return false;
    }
    
    if( rf.check("mode") ) {
        mode = rf.find("mode").asString();
    } else {
        yError("[quaternionEKFModule::configure] Configuration failed. No value for mode was found.");
        return false;
    }
    
    if ( rf.check("usingXSens") ) {
        usingxsens = rf.find("usingXSens").asBool();
    } else {
        yError ("[quaternionEKFModule::configure] Configuration failed. No value for usingXSens was found.");
        return false;
    }
    
    if ( rf.check("verbose") ) {
        verbose = rf.find("verbose").asBool();
    } else {
        yError ("[quaternionEKFModule::configure] Configuration failed. No value for verbose was found.");
        return false;
    }
    
    // ------------ IMU PORT ---------------------------------------
    /*TODO This should be configurable! The number of input ports
     depending on the amount of sensor readings.*/
    std::string tmpOffline = "offline";
    std::string tmpOnline  = "online";
    //  If the estimate is done online
    if (!tmpOnline.compare(mode)) {
        yInfo(" [quaternionEKFModule::configure] Online estimation will be performed");
        std::string gyroMeasPortName = "/";
        gyroMeasPortName += "quaternionEKFModule";
        gyroMeasPortName += "/imu:i";
        if (!gyroMeasPort.open(gyroMeasPortName.c_str())) {
            yError("[quaternionEKFModule::configure] Could not open gyroMeasPort");
            return false;
        }

        // Obtaining filter parameters from configuration file
        yarp::os::Property filterParams;
        if( !rf.check(FILTER_GROUP_PARAMS_NAME) )  {
            yError("[quaternionEKFModule::configure] Could not load EKF-PARAMS group from config file");
            return false;
        } else   {
            filterParams.fromString(rf.findGroup(FILTER_GROUP_PARAMS_NAME).tail().toString());
            yInfo(" [quaternionEKFModule::configure] Filter parameters are: %s ", filterParams.toString().c_str());
        }
        
        // ----------- THREAD INSTANTIATION AND CALLING -----------------
        quatEKFThread = new quaternionEKFThread(period, local, robotName, autoconnect, usingxsens, verbose, filterParams, &gyroMeasPort);
        if (!quatEKFThread->start()) {
            yError("Error starting quaternionEKFThread!");
            return false;
        }
        
        yInfo(" [quaternionEKFModule::configure] quaternionEKFThread started");
        
    } else {
        // If the estimate is done offline, read from file with a datadumper format and don't create the thread.
        if(!tmpOffline.compare(mode)) {
            yInfo(" [quaternionEKFModule::configure] Offline batch estimation will be performed");
            
            // **** Initialization
            // Create dataDumper parser
            m_parser = new dataDumperParser(DATAFILE);
            m_parser->parseFile();
            m_parser->countLines();
            
            // Change period of the module thread
            
        } else {
            yError("[quaternionEKFModule::configure] An invalid option was passed to 'mode'. Available options are 'offline' or 'online'.");
            return false;
        }
    }
    
    
  return true;
}

bool quaternionEKFModule::updateModule()
{
    std::string tmp = "offline";
//     std::cout << "Module period" << this->getPeriod() << std::endl;
    if (!tmp.compare(mode)) {
        if(!m_parser->parseLine(m_currentData)) {
            yInfo("[ quaternionEKFThread::run] File was fully processed or it could not be opened. Quitting thread.");
            return false;
        } else {
        // TODO Perform the offline estimates
        }
    } else {
        // TODO Perform the offline estimates
        return true;
    }
    return true;
}

bool quaternionEKFModule::close()
{
    std::string tmp = "online";
    if (!tmp.compare(mode)) {
        if (quatEKFThread) {
            yInfo(" [quaternionEKFModule::close] Closing thread ...");
            quatEKFThread->stop();
            yInfo(" [quaternionEKFModule::close] Thread was stopped ...");
            delete quatEKFThread;
            quatEKFThread = NULL;
            yInfo(" [quaternionEKFModule::close] Thread closed");
        }
        gyroMeasPort.interrupt();
    }
    return true;
}

