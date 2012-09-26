/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace stim300;

Task::Task(std::string const& name, TaskCore::TaskState initial_state)
    : TaskBase(name, initial_state)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : TaskBase(name, engine, initial_state)
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
    return true;
}
void Task::updateHook()
{
    
    stim300_driver.getInfo();
    stim300_driver.processPacket();
    
    if (stim300_driver.getStatus())
    {
 	base::Time recvts = base::Time::now();
 
 	int packet_counter = stim300_driver.getPacketCounter();
 
 	base::Time ts = timestamp_estimator->update(recvts,packet_counter);
        timeout_counter = 0;
	
	base::samples::IMUSensors sensors;

	sensors.time = ts;
	sensors.acc = stim300_driver.getAccData();
	sensors.gyro = stim300_driver.getGyroData();
	
	_calibrated_sensors.write( sensors );
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

