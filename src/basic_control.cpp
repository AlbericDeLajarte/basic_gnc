#include "ros/ros.h"

#include <ros/package.h>

#include "real_time_simulator/FSM.h"
#include "real_time_simulator/State.h"
#include "real_time_simulator/Waypoint.h"
#include "real_time_simulator/Trajectory.h"

#include "real_time_simulator/Control.h"
#include "geometry_msgs/Vector3.h"

#include "real_time_simulator/GetFSM.h"
#include "real_time_simulator/GetWaypoint.h"

#include <time.h>
#include <sstream>
#include <string>

#include <iomanip>
#include <iostream>
#include <chrono>

#include "rocket_model.hpp"

real_time_simulator::Control PD_control();


// Global variable with last received rocket state
real_time_simulator::State current_state;

// Global variable with last requested fsm
real_time_simulator::FSM current_fsm;

// Global variable with rocket parameters and useful methods
Rocket rocket;

// Callback function to store last received state
void rocket_stateCallback(const real_time_simulator::State::ConstPtr& rocket_state)
{
	current_state.pose = rocket_state->pose;
  current_state.twist = rocket_state->twist;
  current_state.propeller_mass = rocket_state->propeller_mass;
}

void fsm_Callback(const real_time_simulator::FSM::ConstPtr& fsm)
{
  current_fsm.state_machine = fsm->state_machine;
  current_fsm.time_now = fsm->time_now;
}



int main(int argc, char **argv)
{
	// Init ROS control node
  ros::init(argc, argv, "control");
  ros::NodeHandle n;

	// Create control publisher
	ros::Publisher control_pub = n.advertise<real_time_simulator::Control>("control_pub", 10);

	// Subscribe to state message from simulation
  ros::Subscriber rocket_state_sub = n.subscribe("kalman_rocket_state", 100, rocket_stateCallback);

  // Subscribe to fsm and time from time_keeper
  ros::Subscriber fsm_sub = n.subscribe("gnc_fsm_pub", 100, fsm_Callback);

	// Setup Time_keeper client and srv variable for FSM and time synchronization
	ros::ServiceClient client_fsm = n.serviceClient<real_time_simulator::GetFSM>("getFSM_gnc");
  real_time_simulator::GetFSM srv_fsm;

  // Setup Waypoint client and srv variable for trajectory following
  ros::ServiceClient client_waypoint = n.serviceClient<real_time_simulator::GetWaypoint>("getWaypoint");
  real_time_simulator::GetWaypoint srv_waypoint;	

	// Initialize fsm
	current_fsm.time_now = 0;
	current_fsm.state_machine = "Idle";
	
  // Initialize rocket class with useful parameters
  rocket.init(n);

  // Thread to compute control. Duration defines interval time in seconds
  ros::Timer control_thread = n.createTimer(ros::Duration(0.05), [&](const ros::TimerEvent&) 
	{
    // Init default control to zero
    real_time_simulator::Control control_law;

    //Get current FSM and time
    if(client_fsm.call(srv_fsm))
    {
      current_fsm = srv_fsm.response.fsm;
    }
   
    // State machine ------------------------------------------
		if (current_fsm.state_machine.compare("Idle") == 0)
		{
			// Do nothing
		}

    else 
    {
      if (current_fsm.state_machine.compare("Rail") == 0)
      {
        control_law = PD_control();
      }

      else if (current_fsm.state_machine.compare("Launch") == 0)
      {

        control_law = PD_control();
      }

      else if (current_fsm.state_machine.compare("Coast") == 0)
      {

      }
    
      control_pub.publish(control_law);
    }
  });

	// Automatic callback of service and publisher from here
	ros::spin();

}


real_time_simulator::Control PD_control()
{
  // Init control message
  real_time_simulator::Control control_law;
  geometry_msgs::Vector3 thrust_force;
	geometry_msgs::Vector3 thrust_torque;

  thrust_force.x = -200*current_state.twist.angular.y;
  thrust_force.y = -200*current_state.twist.angular.x;
  thrust_force.z = rocket.get_full_thrust(current_fsm.time_now);

  thrust_torque.x = thrust_force.y*rocket.total_CM;
  thrust_torque.y = thrust_force.x*rocket.total_CM;
  thrust_torque.z = -10*current_state.twist.angular.z;

  control_law.force = thrust_force;
  control_law.torque = thrust_torque;

  return control_law;
}