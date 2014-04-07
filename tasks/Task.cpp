/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <rtt/extras/FileDescriptorActivity.hpp> //for the fd_driven

#ifndef D2R
#define D2R M_PI/180.00 /** Convert degree to radian **/
#endif
#ifndef R2D
#define R2D 180.00/M_PI /** Convert radian to degree **/
#endif

//#define DEBUG_PRINTS 1

using namespace stim300;

Task::Task(std::string const& name)
    : TaskBase(name)
{
    prev_ts = base::Time::fromSeconds(0.00);
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
    delete timestamp_estimator;
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{

    if (! TaskBase::configureHook())
        return false;

    /*********************************/
    /** Configuration of the driver **/
    /*********************************/
    if (_revision.value() == stim300::REV_B)
        stim300_driver = new stim300::Stim300RevB();
    else if (_revision.value() == stim300::REV_D)
        stim300_driver = new stim300::Stim300RevD();
    else
        throw std::runtime_error("STIM300 Firmware Revision NOT implemented");

    /** Set the baudrate to the value in the rock property */
    stim300_driver->setBaudrate(_baudrate.value());

    /** Set the packageTimeout **/
    stim300_driver->setPackageTimeout((uint64_t)_timeout);

    /** Open the port **/
    if (!stim300_driver->open(_port.value()))
    {
        std::cerr << "Error opening device '" << _port.value() << "'" << std::endl;
        return false;
    }

    /******************************************/
    /** Configuration of the attitude filter **/
    /******************************************/
    Eigen::Matrix< double, IKFSTATEVECTORSIZE , 1  > x_0; /** Initial vector state **/
    Eigen::Matrix3d Ra; /** Measurement noise covariance matrix for acc */
    Eigen::Matrix3d Rg; /** Measurement noise covariance matrix for gyros */
    Eigen::Matrix3d Rm; /** Measurement noise covariance matrix for mag */
    Eigen::Matrix3d Ri; /** Measurement noise covariance matrix for inclinometers */
    Eigen::Matrix <double, IKFSTATEVECTORSIZE, IKFSTATEVECTORSIZE> P_0; /** Initial covariance matrix **/
    Eigen::Matrix3d Qbg; /** Noise for the gyros bias instability **/
    Eigen::Matrix3d Qba; /** Noise for the acc bias instability **/
    Eigen::Matrix3d Qbi; /** Noise for the inclinometers bias instability **/
    double sqrtdelta_t;

    /************************/
    /** Read configuration **/
    /************************/
    config = _filter_configuration.value();
    accnoise = _accelerometer_noise.value();
    gyronoise = _gyroscope_noise.value();
    incnoise = _inclinometer_noise.value();
    adaptiveconfigAcc = _adaptive_config_acc.value();
    adaptiveconfigInc = _adaptive_config_inc.value();
    location = _location.value();

    /*************************************/
    /** Configuration of Time estimator **/
    /*************************************/
    timestamp_estimator = new aggregator::TimestampEstimator(
	base::Time::fromSeconds(20),
	base::Time::fromSeconds(_timeout.value()/1000.00),
	base::Time::fromSeconds(0),
	INT_MAX);

    /*************************/
    /** Noise configuration **/
    /*************************/
    sqrtdelta_t = sqrt(1.0/accnoise.bandwidth); /** Noise depends on frequency bandwidth **/

    Ra = Eigen::Matrix3d::Zero();
    Ra(0,0) = accnoise.resolution[0] + pow(accnoise.randomwalk[0]/sqrtdelta_t,2);
    Ra(1,1) = accnoise.resolution[1] + pow(accnoise.randomwalk[1]/sqrtdelta_t,2);
    Ra(2,2) = accnoise.resolution[2] + pow(accnoise.randomwalk[2]/sqrtdelta_t,2);

    sqrtdelta_t = sqrt(1.0/gyronoise.bandwidth); /** Noise depends on frequency bandwidth **/

    Rg = Eigen::Matrix3d::Zero();
    Rg(0,0) = pow(gyronoise.randomwalk[0]/sqrtdelta_t,2);
    Rg(1,1) = pow(gyronoise.randomwalk[1]/sqrtdelta_t,2);
    Rg(2,2) = pow(gyronoise.randomwalk[2]/sqrtdelta_t,2);

    sqrtdelta_t = sqrt(1.0/incnoise.bandwidth); /** Noise depends on frequency bandwidth **/

    Ri = Eigen::Matrix3d::Zero();
    Ri(0,0) = incnoise.resolution[0] + pow(incnoise.randomwalk[0]/sqrtdelta_t,2);
    Ri(1,1) = incnoise.resolution[1] + pow(incnoise.randomwalk[1]/sqrtdelta_t,2);
    Ri(2,2) = incnoise.resolution[2] + pow(incnoise.randomwalk[2]/sqrtdelta_t,2);

    /** It does not have magnetometers **/
    Rm = Eigen::Matrix3d::Zero();

    /** Noise for error in gyros bias instability **/
    Qbg.setZero();
    Qbg(0,0) = pow(gyronoise.biasinstability[0],2);
    Qbg(1,1) = pow(gyronoise.biasinstability[1],2);
    Qbg(2,2) = pow(gyronoise.biasinstability[2],2);

    /** Noise for error in accelerometers bias instability **/
    Qba.setZero();
    Qba(0,0) = pow(accnoise.biasinstability[0],2);
    Qba(1,1) = pow(accnoise.biasinstability[1],2);
    Qba(2,2) = pow(accnoise.biasinstability[2],2);

    /** Noise for error in inclinometers bias instability **/
    Qbi.setZero();
    Qbi(0,0) = pow(incnoise.biasinstability[0],2);
    Qbi(1,1) = pow(incnoise.biasinstability[1],2);
    Qbi(2,2) = pow(incnoise.biasinstability[2],2);


    /** Initial error covariance **/
    P_0 = Eigen::Matrix <double,IKFSTATEVECTORSIZE,IKFSTATEVECTORSIZE>::Zero();
    P_0.block <3, 3> (0,0) = 1.0e-06 * Eigen::Matrix3d::Identity();//Error quaternion
    P_0.block <3, 3> (3,3) = 1.0e-06 * Eigen::Matrix3d::Identity();//Gyros bias
    P_0.block <3, 3> (6,6) = 1.0e-06 * Eigen::Matrix3d::Identity();//Accelerometers bias
    P_0.block <3, 3> (9,9) = 1.0e-06 * Eigen::Matrix3d::Identity();//Inclinometers bias

    /** Theoretical Gravity **/
    double gravity = GRAVITY;
    if (location.latitude > 0.0 && location.latitude < 90.0)
        gravity = GravityModel (location.latitude, location.altitude);

    /** Initialize the filter, including the adaptive part **/
    myfilter.Init(P_0, Ra, Rg, Rm, Ri, Qbg, Qba, Qbi, gravity, location.dip_angle,
            adaptiveconfigAcc.M1, adaptiveconfigAcc.M2, adaptiveconfigAcc.gamma,
            adaptiveconfigInc.M1, adaptiveconfigInc.M2, adaptiveconfigInc.gamma);

    /** Leveling configuration **/
    init_leveling_samples.resize(3, config.init_leveling_samples);

    /** Set the index to Zero **/
    init_leveling_idx = 0;

    /** Oldomega initial **/
    oldomega.setZero();

    initAttitude = false;

    /** Output variable **/
    orientationOut.invalidate();
    orientationOut.sourceFrame = config.source_frame_name;
    orientationOut.targetFrame = config.target_frame_name;
    orientationOut.orientation.setIdentity();

    #ifdef DEBUG_PRINTS
    std::cout<< "Rg\n"<<Rg<<"\n";
    std::cout<< "Ra\n"<<Ra<<"\n";
    std::cout<< "Rm\n"<<Rm<<"\n";
    std::cout<< "Ri\n"<<Ri<<"\n";
    std::cout<< "P_0\n"<<P_0<<"\n";
    std::cout<< "Qbg\n"<<Qbg<<"\n";
    std::cout<< "Qba\n"<<Qba<<"\n";
    std::cout<< "Qbi\n"<<Qbi<<"\n";
    #endif

   return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;

    // Here, "fd" is the file descriptor of the underlying device
    // it is usually created in configureHook()
    RTT::extras::FileDescriptorActivity* activity =
        getActivity<RTT::extras::FileDescriptorActivity>();
    if (activity)
    {
        activity->watch(stim300_driver->getFileDescriptor());
	activity->setTimeout(_timeout);
    }

    return true;
}
void Task::updateHook()
{
    /** Inertial sensor values **/
    base::samples::IMUSensors imusamples;

    TaskBase::updateHook();

    /** Process the current package **/
    stim300_driver->processPacket();

    /** Time is current time minus the latency **/
    base::Time recvts = base::Time::now() - base::Time::fromMicroseconds(stim300_driver->getPacketLatency());

    int packet_counter = stim300_driver->getPacketCounter();

    base::Time ts = timestamp_estimator->update(recvts,packet_counter);
    base::Time diffTime = ts - prev_ts;

    imusamples.time = ts;
    prev_ts = ts;
    #ifdef DEBUG_PRINTS
    std::cout<<"Delta time[s]: "<<diffTime.toSeconds()<<"\n";
    #endif

    /** Checksum is good: Take the sensor values from the driver **/
    if (stim300_driver->getChecksumStatus())// && stim300_driver->getStatus())
    {
        imusamples.gyro = stim300_driver->getGyroData();
        imusamples.acc = stim300_driver->getAccData();
        imusamples.mag = stim300_driver->getInclData();//!Short term solution: the mag carries inclinometers info (FINAL SOLUTION REQUIRES: e.g. to change IMUSensor base/types)

        if (_use_filter.value())
        {
            /** Attitude filter **/
            if (!initAttitude)
            {
                #ifdef DEBUG_PRINTS
                std::cout<<"** [ORIENT_IKF] Initial Attitude["<<init_leveling_idx<<"]\n";
                #endif

                if (config.use_inclinometers)
                    init_leveling_samples.col(init_leveling_idx) = imusamples.mag;
                else
                    init_leveling_samples.col(init_leveling_idx) = imusamples.acc;

                init_leveling_idx++;

                if (init_leveling_idx >= config.init_leveling_samples)
                {
                    Eigen::Matrix <double,3,1> meansamples, euler;

                    meansamples[0] = init_leveling_samples.row(0).mean();
                    meansamples[1] = init_leveling_samples.row(1).mean();
                    meansamples[2] = init_leveling_samples.row(2).mean();

                    if ((config.init_leveling_samples > 0) && (base::isnotnan(meansamples)) &&(meansamples.norm() < (GRAVITY+GRAVITY_MARGING)))
                    {
                        euler[0] = (double) asin((double)meansamples[1]/ (double)meansamples.norm()); // Roll
                        euler[1] = (double) -atan(meansamples[0]/meansamples[2]); //Pitch
                        euler[2] = 0.00;//Yaw

                        /** Set the initial attitude  **/
                        attitude = Eigen::Quaternion <double> (Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ())*
                            Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX()));

                        if (config.use_samples_as_theoretical_gravity)
                            myfilter.setGravity(meansamples.norm());
                    }
                    else
                    {
                        attitude.setIdentity();
                        #ifdef DEBUG_PRINTS
                        euler[2] = attitude.toRotationMatrix().eulerAngles(2,1,0)[0];//Yaw
                        euler[1] = attitude.toRotationMatrix().eulerAngles(2,1,0)[1];//Pitch
                        euler[0] = attitude.toRotationMatrix().eulerAngles(2,1,0)[2];//Roll
                        #endif
                    }

                    myfilter.setAttitude(attitude);
                    initAttitude = true;

                    #ifdef DEBUG_PRINTS
                    std::cout<< "******** Initial Attitude  *******"<<"\n";
                    std::cout<< "Init Roll: "<<euler[0]*R2D<<" Init Pitch: "<<euler[1]*R2D<<" Init Yaw: "<<euler[2]*R2D<<"\n";
                    #endif
                }
            }
            else
            {
                double delta_t = diffTime.toSeconds();
                Eigen::Vector3d acc, gyro, incl;
                acc = imusamples.acc; gyro = imusamples.gyro; incl = imusamples.mag;

                /** Eliminate Earth rotation **/
                if (config.init_leveling_samples > 0)
                {
                    Eigen::Quaterniond q_body2world = myfilter.getAttitude().inverse();
                    SubtractEarthRotation(gyro, q_body2world, location.latitude);
                    imusamples.gyro = gyro;
                }

                /** Predict **/
                myfilter.predict(gyro, delta_t);

                /** Update/Correction **/
                myfilter.update(acc, true, incl, config.use_inclinometers);

                /** Delta quaternion of this step **/
                deltaquat = attitude.inverse() * myfilter.getAttitude();

                /** Delta quaternion of this step **/
                deltahead = deltaHeading(gyro, oldomega, delta_t);

                //stim300_driver->printInfo();

                /** Output information **/
                this->outputPortSamples(stim300_driver, myfilter, imusamples);

                /** Timestamp estimator status **/
                _timestamp_estimator_status.write(timestamp_estimator->getStatus());
            }
        }
    }
    else
    {
        //std::cout<<"STIM300 Checksum error\n";
        RTT::log(RTT::Fatal)<<"[STIM300] Datagram Checksum ERROR."<<RTT::endlog();
    }
}

void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();

    RTT::extras::FileDescriptorActivity* activity =
        getActivity<RTT::extras::FileDescriptorActivity>();
    if (activity)
    {
        activity->clearAllWatches();
        //set timeout back so we don't timeout on the rtt's pipe
	activity->setTimeout(0);
    }

    stim300_driver->close();

    timestamp_estimator->reset();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();

    stim300_driver->close();
    delete stim300_driver;

    delete timestamp_estimator;
    timestamp_estimator = NULL;
}

Eigen::Quaternion<double> Task::deltaHeading(const Eigen::Vector3d &angvelo, Eigen::Matrix4d &oldomega, const double delta_t)
{
    Eigen::Matrix4d omega;
    Eigen::Quaternion<double> deltahead;

    omega << 0,-angvelo(0), -angvelo(1), -angvelo(2),
		angvelo(0), 0, angvelo(2), -angvelo(1),
		angvelo(1), -angvelo(2), 0, angvelo(0),
		angvelo(2), angvelo(1), -angvelo(0), 0;


    deltahead = deltaQuaternion(angvelo, oldomega, omega, delta_t);

    oldomega = omega;

    return deltahead;
}

void Task::outputPortSamples(stim300::Stim300Base *driver, filter::Ikf<double, true, true> &myfilter, const base::samples::IMUSensors &imusamples)
{
    Eigen::Matrix <double,IKFSTATEVECTORSIZE,IKFSTATEVECTORSIZE> Pk = myfilter.getCovariance();
    base::samples::IMUSensors compensatedSamples;

    /** Temperature sensors **/
    stim300::Temperature tempSensor;
    tempSensor.time = imusamples.time;
    tempSensor.resize(driver->getTempData().size());

    for (size_t i=0; i<tempSensor.size(); ++i)
        tempSensor.temp[i] = base::Temperature::fromCelsius(stim300_driver->getTempData()[i]);
    _temp_sensors.write(tempSensor);


    if (_use_filter.value())
    {
        /** Merge the two delta quaternion **/
        Eigen::AngleAxisd headangle(deltahead);
        Eigen::AngleAxisd deltaangle(deltaquat);
        Eigen::Vector3d scaleangle = deltaangle.angle() * deltaangle.axis();
        scaleangle[2] = (headangle.angle() * headangle.axis())[2];

        /** Update globally attitude **/
        attitude = attitude * Eigen::Quaternion <double> (Eigen::AngleAxisd(scaleangle[2], Eigen::Vector3d::UnitZ())*
                Eigen::AngleAxisd(scaleangle[1], Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(scaleangle[0], Eigen::Vector3d::UnitX()));

        orientationOut.time = imusamples.time;
        //orientationOut.orientation = myfilter.getAttitude();
        orientationOut.orientation = attitude;
        orientationOut.cov_orientation = Pk.block<3,3>(0,0);
        _orientation_samples_out.write(orientationOut);

        compensatedSamples = imusamples;
        compensatedSamples.gyro = imusamples.gyro - myfilter.getGyroBias();//gyros minus bias
        compensatedSamples.acc = imusamples.acc - myfilter.getAccBias() - myfilter.getGravityinBody(); //acc minus bias and gravity
        _calibrated_sensors.write(compensatedSamples);

        #ifdef DEBUG_PRINTS
        Eigen::Vector3d euler;
        euler[2] = orientationOut.orientation.toRotationMatrix().eulerAngles(2,1,0)[0];//Yaw
        euler[1] = orientationOut.orientation.toRotationMatrix().eulerAngles(2,1,0)[1];//Pitch
        euler[0] = orientationOut.orientation.toRotationMatrix().eulerAngles(2,1,0)[2];//Roll
        std::cout<< "Roll: "<<euler[0]*R2D<<" Pitch: "<<euler[1]*R2D<<" Yaw: "<<euler[2]*R2D<<"\n";
        //Eigen::AngleAxisd angleaxis(orientationOut.orientation);
        //euler = angleaxis.angle() * angleaxis.axis();
        //std::cout<< "Roll: "<<euler[0]*R2D<<" Pitch: "<<euler[1]*R2D<<" Yaw: "<<euler[2]*R2D<<"\n";
        #endif
    }
    else
    {
        _calibrated_sensors.write(imusamples);
    }

}



