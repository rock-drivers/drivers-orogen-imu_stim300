/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <rtt/extras/FileDescriptorActivity.hpp> //for the fd_driven

/** Math only use in the cpp **/
#include <math.h>
#include <fstream>

#ifndef D2R
#define D2R M_PI/180.00 /** Convert degree to radian **/
#endif
#ifndef R2D
#define R2D 180.00/M_PI /** Convert radian to degree **/
#endif

//#define DEBUG_PRINTS 1

using namespace imu_stim300;

Task::Task(std::string const& name)
    : TaskBase(name)
{
    prev_ts = base::Time::fromSeconds(0.00);
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
    prev_ts = base::Time::fromSeconds(0.00);
}

Task::~Task()
{
    if (timestamp_estimator != NULL)
    {
        delete timestamp_estimator;
        timestamp_estimator = NULL;
    }
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
    if (_revision.value() == imu_stim300::REV_B)
        imu_stim300_driver = new imu_stim300::Stim300RevB();
    else if (_revision.value() == imu_stim300::REV_D)
        imu_stim300_driver = new imu_stim300::Stim300RevD();
    else
        throw std::runtime_error("STIM300 Firmware Revision NOT implemented");

    /** Set the baudrate to the value in the rock property */
    imu_stim300_driver->setBaudrate(_baudrate.value());

    /** Set the packageTimeout **/
//    imu_stim300_driver->setPackageTimeout(_timeout.value());
    imu_stim300_driver->setPackageTimeout(0.1);

    /** Calculate the sampling frequency **/
    sampling_frequency = 1.0/base::Time::fromSeconds(_timeout.value()).toSeconds();

    /** Set the frequency **/
    imu_stim300_driver->setFrequency(static_cast<uint64_t>(sampling_frequency));

    /** Open the port **/
    if (!imu_stim300_driver->open(_port.value()))
    {
        std::cerr << "Error opening device '" << _port.value() << "'" << std::endl;
        return false;
    }

    /******************************************/
    /** Configuration of the attitude filter **/
    /******************************************/
    Eigen::Matrix< double, IKFSTATEVECTORSIZE , 1  > x_0; /** Initial vector state **/
    Eigen::Matrix3d Ra; /** Measurement noise covariance matrix for accelerometers */
    Eigen::Matrix3d Rg; /** Measurement noise covariance matrix for gyros */
    Eigen::Matrix3d Rm; /** Measurement noise covariance matrix for mag */
    Eigen::Matrix3d Ri; /** Measurement noise covariance matrix for inclinometers */
    Eigen::Matrix <double, IKFSTATEVECTORSIZE, IKFSTATEVECTORSIZE> P_0; /** Initial covariance matrix **/
    Eigen::Matrix3d Qbg; /** Noise for the gyros bias instability **/
    Eigen::Matrix3d Qba; /** Noise for the accelerometers bias instability **/
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
	base::Time::fromSeconds(1.0/sampling_frequency),
	base::Time::fromSeconds(0),
	INT_MAX);

    /********************************/
    /** Configuration frequencies  **/
    /********************************/
    if (config.correction_frequency > sampling_frequency)
    {
        config.correction_frequency = sampling_frequency;
        RTT::log(RTT::Warning)<<"[STIM300 FILTER] Set  correction frequency to sampling frequency. It cannot be higher that it!!"<<RTT::endlog();
    }

    /******************************/
    /** Correction configuration **/
    /******************************/
    correction_numbers = ceil(sampling_frequency/config.correction_frequency);
    correctionAcc.setZero(); correctionInc.setZero();

    /*************************/
    /** Noise configuration **/
    /*************************/
    if (config.correction_frequency > accnoise.bandwidth)
        sqrtdelta_t = sqrt(1.0/accnoise.bandwidth); /** Noise depends on frequency bandwidth **/
    else
        sqrtdelta_t = sqrt(1.0/config.correction_frequency); /** Noise depends on frequency bandwidth **/

    Ra = Eigen::Matrix3d::Zero();
    Ra(0,0) = accnoise.resolution[0] + pow(accnoise.randomwalk[0]/sqrtdelta_t,2);
    Ra(1,1) = accnoise.resolution[1] + pow(accnoise.randomwalk[1]/sqrtdelta_t,2);
    Ra(2,2) = accnoise.resolution[2] + pow(accnoise.randomwalk[2]/sqrtdelta_t,2);

    sqrtdelta_t = sqrt(1.0/gyronoise.bandwidth); /** Noise depends on frequency bandwidth **/

    Rg = Eigen::Matrix3d::Zero();
    Rg(0,0) = pow(gyronoise.randomwalk[0]/sqrtdelta_t,2);
    Rg(1,1) = pow(gyronoise.randomwalk[1]/sqrtdelta_t,2);
    Rg(2,2) = pow(gyronoise.randomwalk[2]/sqrtdelta_t,2);

    if (config.correction_frequency > incnoise.bandwidth)
        sqrtdelta_t = sqrt(1.0/incnoise.bandwidth); /** Noise depends on frequency bandwidth **/
    else
        sqrtdelta_t = sqrt(1.0/config.correction_frequency); /** Noise depends on frequency bandwidth **/

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
    initial_alignment_gyro.resize(3, config.initial_alignment_samples);
    initial_alignment_acc.resize(3, config.initial_alignment_samples);

    /** Set the index to Zero **/
    initial_alignment_idx = 0;

    /** Set the correction index **/
    correction_idx = 0;

    /** Oldomega initial **/
    oldomega.setZero();

    /** Initial attitude **/
    initAttitude = false;

    /** Output variable **/
    orientation_out.invalidate();
    orientation_out.sourceFrame = config.source_frame_name;
    orientation_out.targetFrame = config.target_frame_name;
    orientation_out.orientation.setIdentity();

    #ifdef DEBUG_PRINTS
    std::cout<< "Sampling frequency: "<<sampling_frequency<<"\n";
    std::cout<< "Correction frequency: "<<config.correction_frequency<<"\n";
    std::cout<< "Correction numbers: "<<correction_numbers<<"\n";
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

    RTT::extras::FileDescriptorActivity* fd_activity =
        getActivity<RTT::extras::FileDescriptorActivity>();

    // Here, imu_stim300_driver->getFileDescriptor() is the file descriptor of
    // the underlying device it is usually created in configureHook()
    if (fd_activity)
    {
        fd_activity->watch(imu_stim300_driver->getFileDescriptor());
//	    fd_activity->setTimeout(_timeout * 1000.00);
	    fd_activity->setTimeout(0.1 * 1000.00);
    }

    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();

    /** Inertial sensor values **/
    base::samples::IMUSensors imusamples;

    RTT::extras::FileDescriptorActivity* fd_activity =
        getActivity<RTT::extras::FileDescriptorActivity>();

    if (!fd_activity)
    {
        throw std::runtime_error("[ERROR] No RTT fd_activity is activated");
        return;
    }

    if (fd_activity->hasError())
    {
        RTT::log(RTT::Fatal)<<"[STIM300] ERROR in fd_activity."<<RTT::endlog();
    }
    else if (fd_activity->hasTimeout())
    {
        RTT::log(RTT::Info)<<"[STIM300] Timeout in fd_activity."<<RTT::endlog();
    }
    else
    {

        /** Process the current package **/
        imu_stim300_driver->processPacket();

        /** Time is current time minus the latency **/
        base::Time recvts = base::Time::now();// - base::Time::fromMicroseconds(imu_stim300_driver->getPacketLatency());

        /** Package counter incrementation **/
        int64_t packet_counter = imu_stim300_driver->getPacketCounter();
        _packet_counter.write(packet_counter);

        base::Time ts = timestamp_estimator->update(recvts, packet_counter);
        base::Time diffTime = ts - prev_ts;

        imusamples.time = ts;
        prev_ts = ts;
        #ifdef DEBUG_PRINTS
        std::cout<<"Delta time[s]: "<<diffTime.toSeconds()<<"\n";
        #endif

        /** Checksum is good: Take the sensor values from the driver **/
        if (imu_stim300_driver->getChecksumStatus())// && imu_stim300_driver->getStatus())
        {
            imusamples.gyro = imu_stim300_driver->getGyroData();
            imusamples.acc = imu_stim300_driver->getAccData();
            imusamples.mag = imu_stim300_driver->getInclData();//!Short term solution: the mag carries inclinometers info (FINAL SOLUTION REQUIRES: e.g. to change IMUSensor base/types)

            if (_use_filter.value())
            {
                /** Attitude filter **/
                if (!initAttitude)
                {
                    #ifdef DEBUG_PRINTS
                    std::cout<<"** [ORIENT_IKF] Initial Attitude["<<initial_alignment_idx<<"]\n";
                    #endif

                    if (state() != INITIAL_ALIGNMENT)
                        state(INITIAL_ALIGNMENT);

                    if (config.use_inclinometers)
                        initial_alignment_acc.col(initial_alignment_idx) = imusamples.mag;
                    else
                        initial_alignment_acc.col(initial_alignment_idx) = imusamples.acc;

                    initial_alignment_gyro.col(initial_alignment_idx) = imusamples.gyro;

                    initial_alignment_idx++;

                    /** Calculate the initial alignment to the local geographic frame **/
                    if (initial_alignment_idx >= config.initial_alignment_samples)
                    {

                        /** Set attitude to identity **/
                        attitude.setIdentity();

                        if (config.initial_alignment_samples > 0)
                        {
                            Eigen::Matrix <double,3,1> meanacc, meangyro;

                            /** Acceleration **/
                            meanacc[0] = initial_alignment_acc.row(0).mean();
                            meanacc[1] = initial_alignment_acc.row(1).mean();
                            meanacc[2] = initial_alignment_acc.row(2).mean();

                            /** Angular velocity **/
                            meangyro[0] = initial_alignment_gyro.row(0).mean();
                            meangyro[1] = initial_alignment_gyro.row(1).mean();
                            meangyro[2] = initial_alignment_gyro.row(2).mean();

                            if ((base::isnotnan(meanacc)) && (base::isnotnan(meangyro)))
                            {
                                if (meanacc.norm() < (GRAVITY+GRAVITY_MARGIN) && meanacc.norm() > (GRAVITY-GRAVITY_MARGIN))
                                {
                                    Eigen::Matrix <double,3,1> euler;
                                    Eigen::Matrix3d initialM;

                                    /** Override the gravity model value with the sensed from the sensors **/
                                    if (config.use_samples_as_theoretical_gravity)
                                        myfilter.setGravity(meanacc.norm());

                                    /**********************************************************************************************************************/
                                    /** Commented the code for initial estimation of attitude. Using the given initial heading from config value instead **/
                                    /**********************************************************************************************************************/

//                                    /** Compute the local horizontal plane **/
//                                    euler[0] = (double) asin((double)meanacc[1]/ (double)meanacc.norm()); // Roll
//                                    euler[1] = (double) -atan(meanacc[0]/meanacc[2]); //Pitch
//                                    euler[2] = 0.00; //Yaw
//
//                                    /** Set the attitude  **/
//                                    attitude = Eigen::Quaternion <double> (Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ())*
//                                        Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) *
//                                        Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX()));
//
//                                    #ifdef DEBUG_PRINTS
//                                    std::cout<< "******** Local Horizontal *******"<<"\n";
//                                    std::cout<< "Roll: "<<euler[0]*R2D<<" Pitch: "<<euler[1]*R2D<<" Yaw: "<<euler[2]*R2D<<"\n";
//                                    #endif
//
//                                    /** The angular velocity in the local horizontal plane **/
//                                    /** Gyro_ho = Tho_body * gyro_body **/
//                                    meangyro = attitude * meangyro;
//
//                                    /** Determine the heading or azimuthal orientation **/
//                                    if (meangyro[0] == 0.00)
//                                        euler[2] = 90.0*D2R - atan(meangyro[0]/meangyro[1]);
//                                    else
//                                        euler[2] = atan(meangyro[1]/meangyro[0]);
//
//                                    /** Set the attitude  **/
//                                    attitude = Eigen::Quaternion <double> (Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ())*
//                                        Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) *
//                                        Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX()));
//
//                                    #ifdef DEBUG_PRINTS
//                                    std::cout<< " Mean Gyro:\n"<<meangyro<<"\n Mean Acc:\n"<<meanacc<<"\n";
//                                    std::cout<< " Earth rot * cos(lat): "<<EARTHW*cos(location.latitude)<<"\n";
//                                    std::cout<< " Filter Gravity: "<<myfilter.getGravity()[2]<<"\n";
//                                    std::cout<< "******** Azimuthal Orientation *******"<<"\n";
//                                    std::cout<< " Yaw: "<<euler[2]*R2D<<"\n";
//                                    #endif
//
//                                    /** Compute the Initial Bias **/
//                                    meangyro[0] = initial_alignment_gyro.row(0).mean();
//                                    meangyro[1] = initial_alignment_gyro.row(1).mean();
//                                    meangyro[2] = initial_alignment_gyro.row(2).mean();

                                    /****************************************************************************************/
                                    /** End of commented block for initial estimation of attitude. Start of inserted block **/
                                    /****************************************************************************************/

                                    /** Compute the initial attitude for Roll and Pitch and use the config value for Yaw **/
                                    euler[0] = (double) asin((double)meanacc[1]/ (double)meanacc.norm()); // Roll
                                    euler[1] = (double) -atan(meanacc[0]/meanacc[2]); //Pitch
                                    euler[2] = _init_heading.value()*D2R; //Yaw

                                    /** Set the attitude  **/
                                    attitude = Eigen::Quaternion <double> (Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ())*
                                        Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) *
                                        Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX()));

                                    /********************************************************/
                                    /** End of inserted block for setting initial attitude **/
                                    /********************************************************/

                                    Eigen::Quaterniond q_body2world = attitude.inverse();
                                    SubtractEarthRotation(meangyro, q_body2world, location.latitude);
                                    meanacc = meanacc - q_body2world * myfilter.getGravity();

                                    if (_use_input_bias.value())
                                    {
                                        std::vector<double> biasacc=_biasacc.value();
                                        std::vector<double> biasgyro=_biasgyro.value();

                                        for (int i=0;i<3;i++)
                                        {
                                            meanacc[i]=biasacc[i];
                                            meangyro[i]=biasgyro[i];
                                        }
                                    }

                                    if (config.use_inclinometers)
                                        myfilter.setInitBias (meangyro, Eigen::Matrix<double, 3, 1>::Zero(), meanacc);
                                    else
                                        myfilter.setInitBias (meangyro, meanacc, Eigen::Matrix<double, 3, 1>::Zero());

                                    #ifdef DEBUG_PRINTS
                                    std::cout<< "******** Initial Bias Offset *******"<<"\n";
                                    std::cout<< " Gyroscopes Bias Offset:\n"<<myfilter.getGyroBias()<<"\n";
                                    std::cout<< " Accelerometers Bias Offset:\n"<<myfilter.getAccBias()<<"\n";
                                    std::cout<< " Inclinometers Bias Offset:\n"<<myfilter.getInclBias()<<"\n";
                                    #endif

                                    /** output estimated bias values to a file **/
                                    char filename[240];
                                    sprintf (filename, _bias_estimation_file.value().c_str());
                                    std::ofstream bias_estimation;
                                    bias_estimation.open(filename);
                                    bias_estimation<< "******** Initial Bias Offset *******"<<"\n";
                                    bias_estimation<< " Gyroscopes Bias Offset:\n"<<myfilter.getGyroBias()<<"\n";
                                    bias_estimation<< " Accelerometers Bias Offset:\n"<<myfilter.getAccBias()<<"\n";
                                    bias_estimation<< " Inclinometers Bias Offset:\n"<<myfilter.getInclBias()<<"\n";
                                    
                                }
                                else
                                {
                                    RTT::log(RTT::Fatal)<<"[STIM300] ERROR in Initial Alignment. Unable to compute reliable attitude."<<RTT::endlog();
                                    RTT::log(RTT::Fatal)<<"[STIM300] Computed "<< meanacc.norm() <<" [m/s^2] gravitational margin of "<<GRAVITY_MARGIN<<" [m/s^2] has been exceeded."<<RTT::endlog();
                                    return exception(ALIGNMENT_ERROR);
                                }
                            }
                            else
                            {
                                RTT::log(RTT::Fatal)<<"[STIM300] ERROR - NaN values in Initial Alignment."<<RTT::endlog();
                                RTT::log(RTT::Fatal)<<"[STIM300] This might be a configuration error or sensor fault."<<RTT::endlog();
                                return exception(NAN_ERROR);
                            }
                        }

                        myfilter.setAttitude(attitude);
                        initAttitude = true;

                        #ifdef DEBUG_PRINTS
                        Eigen::Matrix <double,3,1> eulerprint = base::getEuler(attitude);
                        std::cout<< "******** Initial Attitude  *******"<<"\n";
                        std::cout<< "Init Roll: "<<eulerprint[2]*R2D<<" Init Pitch: "<<eulerprint[1]*R2D<<" Init Yaw: "<<eulerprint[0]*R2D<<"\n";
                        #endif
                    }
                }
                else
                {
                    double delta_t = diffTime.toSeconds();
                    Eigen::Vector3d acc, gyro, inc;
                    acc = imusamples.acc; gyro = imusamples.gyro; inc = imusamples.mag;

                    if (state() != RUNNING)
                        state(RUNNING);

                    #ifdef DEBUG_PRINTS
                    struct timeval start, end;
                    gettimeofday(&start, NULL);
                    #endif

                    /** Eliminate Earth rotation **/
                    Eigen::Quaterniond q_body2world = myfilter.getAttitude().inverse();
                    SubtractEarthRotation(gyro, q_body2world, location.latitude);

                    /** Predict **/
                    myfilter.predict(gyro, delta_t);

                    /** Accumulate correction measurements **/
                    correctionAcc += acc; correctionInc += inc;
                    correction_idx++;
                    #ifdef DEBUG_PRINTS
                    std::cout<<"correction index: "<<correction_idx<<"\n";
                    #endif

                    if (correction_idx == correction_numbers)
                    {
                        acc = correctionAcc / correction_numbers;
                        inc = correctionInc / correction_numbers;

                        #ifdef DEBUG_PRINTS
                        std::cout<<"UPDATE\n";
                        std::cout<<"acc\n"<<acc<<"\n";
                        std::cout<<"inc\n"<<inc<<"\n";
                        #endif

                        /** Update/Correction **/
                        myfilter.update(acc, true, inc, config.use_inclinometers);

                        correctionAcc.setZero();
                        correctionInc.setZero();
                        correction_idx = 0.00;
                    }

                    #ifdef DEBUG_PRINTS
                    gettimeofday(&end, NULL);

                    double execution_delta = ((end.tv_sec  - start.tv_sec) * 1000000u + end.tv_usec - start.tv_usec) / 1.e6;
                    std::cout<<"Execution delta:"<< execution_delta<<"\n";

                    imu_stim300_driver->printInfo();
                    #endif

                    /** Timestamp estimator status **/
                    _timestamp_estimator_status.write(timestamp_estimator->getStatus());
                }
            }

            /** Output information **/
            this->outputPortSamples(imu_stim300_driver, myfilter, imusamples);
        }
        else
        {
            //std::cout<<"STIM300 Checksum error\n";
            RTT::log(RTT::Fatal)<<"[STIM300] Datagram Checksum ERROR."<<RTT::endlog();
        }
    }
}

void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();

    RTT::extras::FileDescriptorActivity* fd_activity =
        getActivity<RTT::extras::FileDescriptorActivity>();

    if (fd_activity)
    {
        fd_activity->clearAllWatches();

        //set timeout back so we don't timeout on the rtt's pipe
        fd_activity->setTimeout(0);
    }

    imu_stim300_driver->close();

    timestamp_estimator->reset();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();

    imu_stim300_driver->close();
    delete imu_stim300_driver;

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

void Task::outputPortSamples(imu_stim300::Stim300Base *driver, filter::Ikf<double, true, true> &myfilter, base::samples::IMUSensors &imusamples)
{
    Eigen::Matrix <double,IKFSTATEVECTORSIZE,IKFSTATEVECTORSIZE> Pk = myfilter.getCovariance();

    /** Temperature sensors **/
    imu_stim300::Temperature tempSensor;
    tempSensor.time = imusamples.time;
    tempSensor.resize(driver->getTempData().size());

    /** Temperature on the International Units **/
    for (size_t i=0; i<tempSensor.size(); ++i)
        tempSensor.temp[i] = base::Temperature::fromCelsius(imu_stim300_driver->getTempData()[i]);

    _temp_sensors_out.write(tempSensor);

    /** Raw calibrated inertial sensor **/
    _inertial_sensors_out.write(imusamples);

    if ((_use_filter.value()) && (state() == RUNNING))
    {
        orientation_out.time = imusamples.time;
        orientation_out.orientation = myfilter.getAttitude();
        orientation_out.cov_orientation = Pk.block<3,3>(0,0);
        _orientation_samples_out.write(orientation_out);

        /** Compensated and calibrated inertial sensor **/
        Eigen::Vector3d gyro = imusamples.gyro;
        SubtractEarthRotation(gyro, orientation_out.orientation.inverse(), location.latitude); //gyros minus Earth rotation
        imusamples.gyro = gyro - myfilter.getGyroBias();//gyros minus bias
        imusamples.acc = imusamples.acc - myfilter.getAccBias() - myfilter.getGravityinBody(); //acc minus bias and gravity
        imusamples.mag = imusamples.mag - myfilter.getInclBias() - myfilter.getGravityinBody(); //inclinometers minus bias and gravity
        _compensated_sensors_out.write(imusamples);

        #ifdef DEBUG_PRINTS
        Eigen::Vector3d euler = base::getEuler(orientation_out.orientation);
        std::cout<< "Roll: "<<euler[2]*R2D<<" Pitch: "<<euler[1]*R2D<<" Yaw: "<<euler[0]*R2D<<"\n";
        //Eigen::AngleAxisd angleaxis(orientation_out.orientation);
        //euler = angleaxis.angle() * angleaxis.axis();
        //std::cout<< "Roll: "<<euler[0]*R2D<<" Pitch: "<<euler[1]*R2D<<" Yaw: "<<euler[2]*R2D<<"\n";
        #endif
    }

}



