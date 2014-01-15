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

    timestamp_estimator = new aggregator::TimestampEstimator(
	base::Time::fromSeconds(20),
	base::Time::fromSeconds(1.0 / stim300::DEFAULT_SAMPLING_FREQUENCY),
	base::Time::fromSeconds(0),
	INT_MAX);

    /** Set the driver **/
    if (_revision.value() == stim300::REV_B)
        stim300_driver = new stim300::Stim300RevB();
    else if (_revision.value() == stim300::REV_D)
        stim300_driver = new stim300::Stim300RevD();
    else
        throw std::runtime_error("STIM300 Firmware Revision NO implemented");

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

    //stim300_driver->printInfo();
    stim300_driver->processPacket();

    if (stim300_driver->getStatus())
    {
        /** Time is current time minus the latency **/
 	base::Time recvts = base::Time::now() - base::Time::fromMicroseconds(stim300_driver->getPacketLatency());

 	int packet_counter = stim300_driver->getPacketCounter();

 	base::Time ts = timestamp_estimator->update(recvts,packet_counter);

	base::samples::IMUSensors sensors;

	sensors.time = ts;
        sensors.acc = stim300_driver->getAccData();
        sensors.mag = stim300_driver->getInclData();//!Short term solution: the mag carries inclinometers info (FINAL SOLUTION REQUIRES: e.g. to change IMUSensor base/types)
	
	sensors.gyro = stim300_driver->getGyroData();
	
	_calibrated_sensors.write(sensors);
	
	
	stim300::Temperature tempSensor;
	tempSensor.time = ts;
	tempSensor.resize(stim300_driver->getTempData().size());

        for (size_t i=0; i<tempSensor.size(); ++i)
            tempSensor.temp[i] = base::Temperature::fromCelsius(stim300_driver->getTempData()[i]);
	_temp_sensors.write(tempSensor);

        if (!stim300_driver->getChecksumStatus())
            RTT::log(RTT::Fatal)<<"[STIM300] Datagram Checksum ERROR."<<RTT::endlog();
	
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

