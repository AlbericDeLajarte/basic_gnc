/*
* Node to synchronize the GNC algorithms by estimating the current state of the rocket
* (Idle, Rail, Launch, Coast)
*
* Inputs: 
*   - Sensor data (IMU and barometer):					\sensor_pub
*   - Estimated state from basic_navigation:		    \kalman_rocket_state
*
* Parameters:
*   - Threshold for rocket ignition detection (Rail phase): in FSM_thread
*
* Outputs:
*   - Estimated finite state machine from flight data:	\gnc_fsm_pub
*
*/

#include "ros/ros.h"
#include "real_time_simulator/FSM.h"
#include "real_time_simulator/State.h"
#include "real_time_simulator/Sensor.h"

#include <time.h>

#include <sstream>
#include <string>

#include "real_time_simulator/GetFSM.h"
#include "std_msgs/String.h"

// global variable with time and state machine
real_time_simulator::FSM current_fsm;
double time_zero;

// Global variable with last received sensor data
real_time_simulator::Sensor current_sensor;

// global variable with last received rocket state
real_time_simulator::State current_rocket_state;

void rocket_stateCallback(const real_time_simulator::State::ConstPtr& rocket_state)
{
	current_rocket_state.pose = rocket_state->pose;
  	current_rocket_state.twist = rocket_state->twist;
 	current_rocket_state.propeller_mass = rocket_state->propeller_mass;
}

// Callback function to store last received sensor data
void sensorCallback(const real_time_simulator::Sensor::ConstPtr& sensor)
{
	current_sensor.IMU_acc = sensor->IMU_acc;
	current_sensor.IMU_gyro = sensor->IMU_gyro;
	current_sensor.baro_height = sensor->baro_height;
}

// Service function: send back fsm (time + state machine)
bool sendFSM(real_time_simulator::GetFSM::Request &req, real_time_simulator::GetFSM::Response &res)
{
	// Update current time
	if (current_fsm.state_machine.compare("Idle") != 0) current_fsm.time_now = ros::Time::now().toSec() - time_zero;

	res.fsm.time_now = current_fsm.time_now;
	res.fsm.state_machine = current_fsm.state_machine;
	
	return true;
}


float rail_length = 0;

int main(int argc, char **argv)
{

	// Init ROS time keeper node
	ros::init(argc, argv, "basic_fsm");
	ros::NodeHandle n;

	// Initialize fsm
	current_fsm.time_now = 0;
	current_fsm.state_machine = "Idle";

	// Initialize rocket state
	current_rocket_state.propeller_mass = 10; // To stay in launch mode at first iteration

	// Create timer service
	ros::ServiceServer timer_service = n.advertiseService("getFSM_gnc", sendFSM);

	// Create timer publisher and associated thread (100Hz)
	ros::Publisher timer_pub = n.advertise<real_time_simulator::FSM>("gnc_fsm_pub", 10);

	// Subscribe to state message
	ros::Subscriber rocket_state_sub = n.subscribe("kalman_rocket_state", 100, rocket_stateCallback);

	// Subscribe to sensors message
	ros::Subscriber sensor_sub = n.subscribe("sensor_pub", 100, sensorCallback);

	timer_pub.publish(current_fsm);

	n.getParam("/environment/rail_length", rail_length);

	ros::Timer FSM_thread = n.createTimer(ros::Duration(0.01),
	[&](const ros::TimerEvent&) 
	{
		// Update FSM
		if (current_fsm.state_machine.compare("Idle") == 0)
		{
			if(current_sensor.IMU_acc.z > 30)
			{
				current_fsm.state_machine = "Rail";
				time_zero = ros::Time::now().toSec();
			}
		}

		else
		{
			// Update current time
			current_fsm.time_now = ros::Time::now().toSec() - time_zero;
			
			if (current_fsm.state_machine.compare("Rail") == 0)
			{
				// End of rail
				if(current_rocket_state.pose.position.z > rail_length)
				{
					current_fsm.state_machine = "Launch";
				}

			}

			else if (current_fsm.state_machine.compare("Launch") == 0)
			{
				// End of burn -> no more thrust
				if(current_rocket_state.propeller_mass < 0)
				{
					current_fsm.state_machine = "Coast";
				}

			}

			else if (current_fsm.state_machine.compare("Coast") == 0)
			{
			// Do nothing for now
			}

			// Publish time + state machine    
			timer_pub.publish(current_fsm);
		}

	});

	// Automatic callback of service and publisher from here
	ros::spin();

}
