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
                                           bool usingxsens,
                                           bool verbose,
                                           yarp::os::Property &filterParams,
                                           yarp::os::BufferedPort<yarp::sig::Vector>* gyroMeasPort
                                         )
    : RateThread ( period ),
      m_period ( period ),
      m_moduleName ( moduleName ),
      m_robotName ( robotName ),
      m_autoconnect ( autoconnect ),
      m_usingxsens ( usingxsens ),
      m_verbose ( verbose ),
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
//     yarp::sig::Vector* imu_measurement;
    imu_measurement = new yarp::sig::Vector(12);
    if ( !m_usingxsens ) {
        bool reading = true;
        imu_measurement = m_gyroMeasPort->read(reading);
    } else {
        readDataFromXSens(imu_measurement);
    }
    
    if (m_verbose) {
        cout << "Full imu_measurement vec: " << endl;
        cout << imu_measurement->toString().c_str() << endl;
    }
    // XSens orientation
    yarp::sig::Vector realOrientation = imu_measurement->subVector(0,2);
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
    if (m_verbose)
        cout << "VEL INPUT IS: " << input << endl;
    // Extract accelerometer. Read from port!
    MatrixWrapper::ColumnVector measurement(m_measurement_size);
    measurement(1) = imu_linAcc(0);
    measurement(2) = imu_linAcc(1);
    measurement(3) = imu_linAcc(2);
    if (m_verbose)
        cout << "ACC INPUT IS: " << measurement << endl;

    // Noise gaussian
    // System Noise Mean
    // TODO This mean changes!!!
    MatrixWrapper::ColumnVector sys_noise_mu(m_state_size);
    sys_noise_mu = 0.0;
    MatrixWrapper::Matrix Xi(m_state_size, m_input_size);
    XiOperator(m_posterior_state, &Xi);
    
    // System Noise Covariance
    // TODO For now let's leave this constant as something to be tuned. 
    // This covariance matrix however should be computed as done in the matlab
    // utility ekfukf/lti_disc.m through Matrix Fraction Decomposition.
    MatrixWrapper::SymmetricMatrix sys_noise_cov(m_state_size);
    sys_noise_cov = 0.0;

    MatrixWrapper::Matrix Sigma_gyro(m_input_size,m_input_size);
    Sigma_gyro = 0.0;
    Sigma_gyro(1,1) = Sigma_gyro(2,2) = Sigma_gyro(3,3) = m_sigma_gyro;

    MatrixWrapper::Matrix tmp = Xi*Sigma_gyro*Xi.transpose();
    MatrixWrapper::SymmetricMatrix tmpSym(m_state_size);
    tmp.convertToSymmetricMatrix(tmpSym);
    sys_noise_cov = (MatrixWrapper::SymmetricMatrix) tmpSym*pow(m_period/(1000.0*2.0),2);
//     sys_noise_cov(1,1) = sys_noise_cov (2,2) = sys_noise_cov(3,3) = sys_noise_cov(4,4) = 0.000001;
    
    if (m_verbose)
        cout << "System covariance matrix will be: " << sys_noise_cov << endl;

    m_sysPdf.AdditiveNoiseMuSet(sys_noise_mu);
    m_sysPdf.AdditiveNoiseSigmaSet(sys_noise_cov);

    double elapsedTime = yarp::os::Time::now() - m_waitingTime;
    
//     double intpart = 0.0;
    
//     // NOTE Let's include the measurement roughly every ten seconds
//     if (modf(elapsedTime/10, &intpart) < 0.001) {
//         if(!m_filter->Update(m_sys_model, input, m_meas_model, measurement))
//             yError(" [quaternionEKFThread::run] Update step of the Kalman Filter could not be performed\n");
//     } else {
//             if(!m_filter->Update(m_sys_model, input))
//                 yError(" [quaternionEKFThread::run] Update step of the Kalman Filter could not be performed\n");
//     }
    
    if(!m_filter->Update(m_sys_model, input, m_meas_model, measurement))
        yError(" [quaternionEKFThread::run] Update step of the Kalman Filter could not be performed\n");
    
    // Get the posterior of the updated filter. Result of all the system model and meaurement information
    BFL::Pdf<BFL::ColumnVector> * posterior = m_filter->PostGet();
    // Posterior Expectation
    m_posterior_state = posterior->ExpectedValueGet();
    MatrixWrapper::Quaternion expectedValueQuat(m_posterior_state);
    // Posterior Covariance
    MatrixWrapper::SymmetricMatrix covariance(m_state_size);
    covariance = posterior->CovarianceGet();
    if (m_verbose) {
        cout << "Posterior Mean: " << expectedValueQuat << endl;
        cout << "Posterior Covariance: " << posterior->CovarianceGet() << endl;
        cout << " " << endl;
    }
    MatrixWrapper::ColumnVector eulerAngles(3);
    expectedValueQuat.getEulerAngles(string("xyz"), eulerAngles);
    if (m_verbose)
        cout << "Posterior Mean in Euler Angles: " << (180/PI)*eulerAngles  << endl;
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
    
    //  Publish XSens orientation just for debugging
    if (m_xsens) {
        yarp::sig::Vector& tmpXSensEuler = m_publisherXSensEuler->prepare();
        tmpXSensEuler = realOrientation;
        m_publisherXSensEuler->write();
    }
    
    cout << "Elapsed time: " << elapsedTime << endl;
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
        m_sigma_gyro = m_filterParams.find("SIGMA_GYRO_NOISE").asDouble();
        m_prior_mu = m_filterParams.find("PRIOR_MU_STATE").asDouble();
        m_prior_cov = m_filterParams.find("PRIOR_COV_STATE").asDouble();
        m_mu_gyro_noise = m_filterParams.find("MU_GYRO_NOISE").asDouble();
        m_smoother = m_filterParams.find("smoother").asBool();
        m_external_imu = m_filterParams.find("externalimu").asBool();
    } else {
        yError(" [quaternionEKFThread::threadInit] Filter parameters from configuration file could not be extracted");
        return false;
    }
    
    // XSens device
    m_xsens = new DeviceClass;
    
    // imu Measurement vector
    imu_measurement = new yarp::sig::Vector(12);
    
    // Open publisher port for estimate in quaternion
    m_publisherFilteredOrientationPort = new yarp::os::BufferedPort<yarp::sig::Vector>;
    m_publisherFilteredOrientationPort->open(string("/" + m_moduleName + "/filteredOrientation:o").c_str());
    
    // Open publisher port for estimate in euler angles
    m_publisherFilteredOrientationEulerPort = new yarp::os::BufferedPort<yarp::sig::Vector>;
    m_publisherFilteredOrientationEulerPort->open(string("/" + m_moduleName + "/filteredOrientationEuler:o").c_str());
    
    if (m_usingxsens) {
        m_publisherXSensEuler = new yarp::os::BufferedPort<yarp::sig::Vector>;
        m_publisherXSensEuler->open(string("/xsens/euler:o").c_str());
    }
    
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
    meas_noise_mu = 0.0;                // Set all to zero
    meas_noise_mu(3) = 0.0;
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

    if (m_autoconnect && !m_usingxsens) {
        yarp::os::ConstString src = std::string("/" + m_robotName + "/inertial");
        if(!yarp::os::Network::connect(src, gyroMeasPortName,"tcp")){
            yError(" [quaternionEKFThread::threadInit()] Connection with %s was not possible. Is the robotInterface running? or XSens IMU connected?", gyroMeasPortName.c_str());
            return false;
        }
    }

    // XSens IMU configuration
    if ( m_usingxsens ) {
        if (!configureXSens()) {
            cout << "XSens configuration was not possible! Check the XSens is plugged to your USB port and that the driver is properly installed" << endl;
            return false;
        }
    }

    cout << "Thread waiting five seconds before starting..." <<  endl;
    yarp::os::Time::delay(5);
    
    
    m_waitingTime = yarp::os::Time::now();
    cout << "Thread is running ... " << endl;
    return true;
}

void quaternionEKFThread::XiOperator ( MatrixWrapper::ColumnVector quat, MatrixWrapper::Matrix* Xi )
{
//     In  Matlab language this would be:
//     Xi = [       -qk(2:4,:)'           ;
//           qk(1)*eye(3) + S(qk(2:4,:)) ];
    (*Xi) = 1.0;
    MatrixWrapper::ColumnVector omg(3);
    omg(1) = quat(2);    omg(2) = quat(3);    omg(3) = quat(4);
    
    (*Xi)(1,1) = -quat(2);    (*Xi)(1,2) = -quat(3);    (*Xi)(1,3) = -quat(4);
    MatrixWrapper::Matrix eye(3,3);
    eye.toIdentity();
    MatrixWrapper::Matrix S(3,3);
    SOperator(omg, &S);
    MatrixWrapper::Matrix tmpAdd = eye*quat(1) + S;
    Xi->setSubMatrix(tmpAdd,2,4,1,3);
    if (m_verbose)
        cout << "Debugging Xi Operator in [quaternionEKFThread::XiOperator]: " << (*Xi) << endl;
}

void quaternionEKFThread::SOperator ( MatrixWrapper::ColumnVector omg, MatrixWrapper::Matrix* S )
{
    (*S)(1,1) = 0.0;    (*S)(1,2) = -omg(3); (*S)(1,3) = omg(2);
    (*S)(2,1) = omg(3); (*S)(2,2) = 0.0    ; (*S)(2,3) = -omg(1);
    (*S)(3,1) = -omg(2);(*S)(3,2) = omg(1) ; (*S)(3,3) = 0.0;
}

bool quaternionEKFThread::configureXSens()
{
    bool ret = false;
    try
    {
        // Scan for connected USB devices
        std::cout << "Scanning for USB devices..." << std::endl;
        XsPortInfoArray portInfoArray;
        xsEnumerateUsbDevices(portInfoArray);
        if (!portInfoArray.size())
        {
            std::string portNumber;
            int baudRate;
            #ifdef WIN32
            std::cout << "No USB Motion Tracker found." << std::endl << std::endl << "Please enter COM port number (eg. 1): " <<
            #else
            std::cout << "No USB Motion Tracker found." << std::endl << std::endl << "Please enter COM port name (eg. /dev/ttyUSB0): " <<
            #endif
            std::endl;
            // TODO Temporarily removed and fixed to /dev/ttyUSB0
//             std::cin >> portNumber;
            portNumber = string("/dev/ttyUSB0");
            // TODO Temporarily removed and fixed to 115200
//             std::cout << "Please enter baud rate (eg. 115200): ";
//             std::cin >> baudRate;
            baudRate = 115200;
            
            XsPortInfo portInfo(portNumber, XsBaud::numericToRate(baudRate));
            portInfoArray.push_back(portInfo);
        }
        
        // Use the first detected device
        m_mtPort = portInfoArray.at(0);
        
        // Open the port with the detected device
        std::cout << "Opening port..." << std::endl;
        if (!m_xsens->openPort(m_mtPort))
            throw std::runtime_error("Could not open port. Aborting.");
        
        // Put the device in configuration mode
        std::cout << "Putting device into configuration mode..." << std::endl;
        if (!m_xsens->gotoConfig()) // Put the device into configuration mode before configuring the device
        {
            throw std::runtime_error("Could not put device into configuration mode. Aborting.");
        }
        
        // Request the device Id to check the device type
        m_mtPort.setDeviceId(m_xsens->getDeviceId());
        
        // Check if we have an MTi / MTx / MTmk4 device
        if (!m_mtPort.deviceId().isMt9c() && !m_mtPort.deviceId().isMtMk4())
        {
            throw std::runtime_error("No MTi / MTx / MTmk4 device found. Aborting.");
        }
        std::cout << "Found a device with id: " << m_mtPort.deviceId().toString().toStdString() << " @ port: " << m_mtPort.portName().toStdString() << ", baudrate: " << m_mtPort.baudrate() << std::endl;
        
        try
        {
            // Print information about detected MTi / MTx / MTmk4 device
            std::cout << "Device: " << m_xsens->getProductCode().toStdString() << " opened." << std::endl;
            
            // Configure the device. Note the differences between MTix and MTmk4
            std::cout << "Configuring the device..." << std::endl;
            uint16_t freq = 100;
            if (m_mtPort.deviceId().isMt9c())
            {
                //TODO HERE In our case we want ORIENTATION | ACCELERATION | ANGULAR VELOCITY. The last two are contained in XOM_Calibrated
                XsOutputMode outputMode = XOM_Orientation | XOM_Calibrated; // output orientation data
                XsOutputSettings outputSettings = XOS_OrientationMode_Euler | XOS_CalibratedMode_All; // output orientation data as quaternion
                
                // set the device configuration
                if (!m_xsens->setDeviceMode(outputMode, outputSettings))
                {
                    throw std::runtime_error("Could not configure MT device. Aborting.");
                }
            }
            else if (m_mtPort.deviceId().isMtMk4())
            {
                XsOutputConfiguration quat(XDI_EulerAngles, freq);
                XsOutputConfigurationArray configArray;
                configArray.push_back(quat);
                
                XsOutputConfiguration acc(XDI_Acceleration, freq);
                configArray.push_back(acc);
                
                XsOutputConfiguration angVel(XDI_RateOfTurn, freq);
                configArray.push_back(angVel);
                
                XsOutputConfiguration magField(XDI_MagneticField, freq);
                configArray.push_back(magField);
                if (!m_xsens->setOutputConfiguration(configArray))
                {
                    
                    throw std::runtime_error("Could not configure MTmk4 device. Aborting.");
                }
            }
            else
            {
                throw std::runtime_error("Unknown device while configuring. Aborting.");
            }
            
            // Put the device in measurement mode
            std::cout << "Putting device into measurement mode..." << std::endl;
            if (!m_xsens->gotoMeasurement())
            {
                throw std::runtime_error("Could not put device into measurement mode. Aborting.");
            }
        }
        catch (std::runtime_error const & error)
        {
            std::cout << error.what() << std::endl;
        }
        catch (...)
        {
            std::cout << "An unknown fatal error has occured. Aborting." << std::endl;
        }
        
    }
    catch (std::runtime_error const & error)
    {
        std::cout << error.what() << std::endl;
    }
    catch (...)
    {
        std::cout << "An unknown fatal error has occured. Aborting." << std::endl;
    }
    
    ret = true;
    return ret;
}

void quaternionEKFThread::readDataFromXSens(yarp::sig::Vector* output)
{
    XsByteArray data;
    XsMessageArray msgs;

    m_xsens->readDataToBuffer(data);
    m_xsens->processBufferedData(data, msgs);
    for (XsMessageArray::iterator it = msgs.begin(); it != msgs.end(); ++it)
    {
        output->clear();
        // Retrieve a packet
        XsDataPacket packet;
        if ((*it).getMessageId() == XMID_MtData) {
            LegacyDataPacket lpacket(1, false);
            lpacket.setMessage((*it));
            lpacket.setXbusSystem(false, false);
            lpacket.setDeviceId(m_mtPort.deviceId(), 0);
            lpacket.setDataFormat(XOM_Orientation | XOM_Calibrated, XOS_OrientationMode_Euler | XOS_CalibratedMode_All,0);       //lint !e534
            XsDataPacket_assignFromXsLegacyDataPacket(&packet, &lpacket, 0);
        }
        else if ((*it).getMessageId() == XMID_MtData2) {
            packet.setMessage((*it));
            packet.setDeviceId(m_mtPort.deviceId());
        }
        
        // Get the quaternion data
        XsQuaternion quaternion = packet.orientationQuaternion();
        if (m_verbose) {
            std::cout << "\r"
            << "W:" << std::setw(5) << std::fixed << std::setprecision(2) << quaternion.m_w
            << ",X:" << std::setw(5) << std::fixed << std::setprecision(2) << quaternion.m_x
            << ",Y:" << std::setw(5) << std::fixed << std::setprecision(2) << quaternion.m_y
            << ",Z:" << std::setw(5) << std::fixed << std::setprecision(2) << quaternion.m_z
            ;
        }
        
        // Convert packet to euler
        XsEuler euler = packet.orientationEuler();
        if (m_verbose) {
            std::cout << ",Roll:" << std::setw(7) << std::fixed << std::setprecision(2) << euler.m_roll
            << ", Pitch:" << std::setw(7) << std::fixed << std::setprecision(2) << euler.m_pitch
            << ", Yaw:" << std::setw(7) << std::fixed << std::setprecision(2) << euler.m_yaw
            <<std::endl;
        }
        
        XsVector linAcc = packet.calibratedAcceleration();
        if (m_verbose) {
            std::cout << "Accelerometer" << std::endl;
            std::cout << "x: " << linAcc[0] << " y: " << linAcc[1] << " z: " << linAcc[2] << std::endl;
        }
        
        XsVector angVel = packet.calibratedGyroscopeData();
        if (m_verbose) {
            std::cout << "Angular velocity" << std::endl;
            std::cout << "x: " << angVel[0] << " y: " << angVel[1] << " z: " << angVel[2] << std::endl;
        }
        
        XsVector magField = packet.calibratedMagneticField();
        if (m_verbose) {
            std::cout << "Magnetic Field" << std::endl;
            std::cout << "x: " << magField[0] << " y: " << magField[1] << " z: " << magField[2] << std::endl;
        }
        
        // Push euler angles
        output->push_back(static_cast<double>(euler.m_roll));
        output->push_back(static_cast<double>(euler.m_pitch));
        output->push_back(static_cast<double>(euler.m_yaw));
        
        // Push lin acc
        output->push_back(static_cast<double>(linAcc[0]));
        output->push_back(static_cast<double>(linAcc[1]));
        output->push_back(static_cast<double>(linAcc[2]));
        
        // Push gyroMeas 
        output->push_back(static_cast<double>(angVel[0]));
        output->push_back(static_cast<double>(angVel[1]));
        output->push_back(static_cast<double>(angVel[2]));
        
        // Push magnetic field
        output->push_back(static_cast<double>(magField[0]));
        output->push_back(static_cast<double>(magField[1]));
        output->push_back(static_cast<double>(magField[2]));
    }
}

void quaternionEKFThread::threadRelease()
{
//     if (m_parser) { 
//         delete m_parser;
//         m_parser = NULL;
//         cout << "m_parser deleted" << endl;
//     }
    if (m_xsens) {
        delete m_xsens;
        m_xsens = NULL;
        cout << "m_xsens deleted" << endl;
    }
    if (m_publisherFilteredOrientationEulerPort) {
        m_publisherFilteredOrientationEulerPort->interrupt();
        delete m_publisherFilteredOrientationEulerPort;
        m_publisherFilteredOrientationEulerPort = NULL;
        cout << "m_publisherFilteredOrientationEulerPort deleted" << endl;
    }
    if (m_publisherFilteredOrientationPort) {
        m_publisherFilteredOrientationPort->interrupt();
        delete m_publisherFilteredOrientationPort;
        m_publisherFilteredOrientationPort = NULL;
        cout << "m_publisherFilteredOrientationPort deleted" << endl;
    }
    if (m_publisherXSensEuler) { 
        m_publisherXSensEuler->interrupt();
        delete m_publisherXSensEuler;
        m_publisherXSensEuler = NULL;
        cout << "m_publisherXSensEuler deleted" << endl;
    }
    if (m_sys_model) {
        delete m_sys_model;
        m_sys_model = NULL;
        cout << "m_sys_model deleted" << endl;
    }
    if (m_measurement_uncertainty) {
        delete m_measurement_uncertainty;
        m_measurement_uncertainty = NULL;
        cout << "m_measurement_uncertainty deleted" << endl;
    }
    if (m_measPdf) {
        delete m_measPdf;
        m_measPdf = NULL;
        cout << "m_measPdf deleted" << endl;
    }
    if (m_meas_model) {
        delete m_meas_model;
        m_meas_model = NULL;
        cout << "m_meas_model deleted" << endl;
    }
    if (m_prior) { 
        delete m_prior;
        m_prior = NULL;
        cout << "m_prior deleted" << endl;
    }
    if (m_filter) {
        delete m_filter;
        m_filter = NULL;
        cout << "m_filter deleted" << endl;
    }
    if (imu_measurement) { 
        delete imu_measurement;
        imu_measurement = NULL;
        cout << "imu_measurement deleted" << endl;
    }
}

