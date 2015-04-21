#include <bfl/filter/extendedkalmanfilter.h>
#include <bfl/model/linearanalyticsystemmodel_gaussianuncertainty.h>
#include <bfl/model/linearanalyticmeasurementmodel_gaussianuncertainty.h>
#include <bfl/pdf/analyticconditionalgaussian.h>
#include <bfl/pdf/linearanalyticconditionalgaussian.h>

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
    rf.setVerbose(true);
    rf.configure(argc, argv);
    
    if(rf.check("help")) {
        yInfo() << "Parameters";
        yInfo() << "\t--from            :Name of .ini file for configuration";
        yInfo() << "\t--robot           :Robot name";
        yInfo() << "\t--rate            :Thread period (ms)";
        return 0;
    }
    
    yarp::os::Network yarpNetwork;
    
    if (!yarpNetwork.checkNetwork())
    {
        yError("YARP Network is not available. The module will shut down now...");
        return -1;
    }
}