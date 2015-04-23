#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>

#include "quaternionEKFModule.h"

#include <iostream>
#include <fstream>

using namespace std;

int main(int argc, char* argv[]) 
{
    yarp::os::ResourceFinder rf;
    rf.setVerbose(false);
    rf.setDefaultConfigFile("quaternionEKFModule.ini");
    rf.configure(argc, argv);
    
    if(rf.check("help")) {
        yInfo() << "Parameters";
        yInfo() << "\t--from            :[quaternionEKFModule.ini] Name of .ini file for configuration";
        yInfo() << "\t--robot           :[icubGazeboSim] Robot name";
        yInfo() << "\t--rate            :[10] Thread period (ms)";
        yInfo() << "\t--mode            :[offline] or online";
        yInfo() << "\t--autoconnect     :[0] or 1";
        return 0;
    }
    
    yarp::os::Network yarpNetwork;
    
    if (!yarpNetwork.checkNetwork())
    {
        yError("YARP Network is not available. The module will shut down now...");
        return -1;
    }
    
    quaternionEKFModule module;
    return module.runModule(rf);
}