/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <rtt/extras/FileDescriptorActivity.hpp> //for the fd_driven

using namespace stim300;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

// Task::Task(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
//     : TaskBase(name, engine, initial_state)
// {
// }

Task::~Task()
{
    delete timestamp_estimator;
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    
    std::string acc_acc ("ACCELERATION");
    std::string acc_vel ("INCREMENTAL_VELOCITY");
     
    if (! TaskBase::configureHook())
        return false;
    
    timestamp_estimator = new aggregator::TimestampEstimator(
	base::Time::fromSeconds(20),
	base::Time::fromSeconds(1.0 / stim300::STIM300Driver::DEFAULT_SAMPLING_FREQUENCY),
	base::Time::fromSeconds(0),
	INT_MAX);
    
    /** Set the baudrate to the value in the rock property *
     * Default is 921600 **/
     stim300_driver.setBaudrate(_baudrate.value());
     
     /** Open the port **/
     if (!stim300_driver.open(_port.value()))
     {
	std::cerr << "Error opening device '" << _port.value() << "'" << std::endl;
        return false;
     }
     
     /** Set the output format for the accelerometers **/
     if (acc_acc.compare(_acc_output.value()) == 0)
     {
	 /** NOTHING BECAUSE THE DEFAULT VALUE IN THE FLASH MEMORY IS ACCELERATION **/
	 /** IF THE FLASH MEMORY IN THE STIM300 IS CHANGED TO ANOTHER DEFAULT VALUE SOMETHING NEED TO BE WRITEN HERE **/
     }
     else if (acc_vel.compare(_acc_output.value()) == 0)
     {
	 /** CHANGE THE OUTPUT TO INCREMENTAL_VELOCITY **/
	 stim300_driver.setAcctoIncrementalVelocity(); 
     }
     
     
    
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
        activity->watch(stim300_driver.getFileDescriptor());
	activity->setTimeout(_timeout);
    }
    return true;
}
void Task::updateHook()
{
    
//     stim300_driver.getInfo();
    stim300_driver.processPacket();
    
    if (stim300_driver.getStatus())
    {
 	base::Time recvts = base::Time::now();
 
 	int packet_counter = stim300_driver.getPacketCounter();
 
 	base::Time ts = timestamp_estimator->update(recvts,packet_counter);
        timeout_counter = 0;
	
	if (stim300_driver.getAccOutputType() == INCREMENTAL_VELOCITY)
	{
	    base::samples::RigidBodyState velocity;
	    velocity.invalidate();
	    velocity.time = ts;
	    velocity.sourceFrame = "IMU Frame (X-Forward, Y-Left, Z-Up)";
	    velocity.targetFrame = "World Frame (N-W-Up)";
	    velocity.velocity = stim300_driver.getAccData();
	    _incremental_velocity.write(velocity);
	}
	
	base::samples::IMUSensors sensors;

	sensors.time = ts;
	if (stim300_driver.getAccOutputType() == ACCELERATION)
	    sensors.acc = stim300_driver.getAccData();
	else
	    sensors.acc = stim300_driver.getInclData();
	
	sensors.gyro = stim300_driver.getGyroData();
	sensors.mag = base::Vector3d::Ones() * base::NaN<double>();
	
	_calibrated_sensors.write(sensors);
	
	stim300::Temperature tempSensor;	
	
	tempSensor.time = ts;
	tempSensor.resize(3);
	
	base::Temperature tempValue;
	tempSensor.temp[0] = tempValue.fromCelsius(stim300_driver.getTempDataX());
	
	tempSensor.temp[1] = tempValue.fromCelsius(stim300_driver.getTempDataY());

	tempSensor.temp[2] = tempValue.fromCelsius(stim300_driver.getTempDataZ());
	
	_temp_sensors.write(tempSensor);
	
    }
    
    TaskBase::updateHook();
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
    
    stim300_driver.close();
    
    timestamp_estimator->reset();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
    
    stim300_driver.close();
    
    delete timestamp_estimator;
    timestamp_estimator = NULL;
}

