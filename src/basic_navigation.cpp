/*
* Node to estimate the rocket full state (position, velocity, quaternion, angular rate and mass) 
* from the sensor data and commanded thrust and torque of the rocket engine
*
* Inputs: 
*   - Finite state machine from the basic_gnc package:	\gnc_fsm_pub
*   - 3D force and torque from the rocket engine:		\control_pub
*   - Sensor data (IMU and barometer):					\sensor_pub
*
* Parameters:
*   - Rocket model: 		/config/rocket_parameters.yaml
*   - Environment model: 	/config/environment_parameters.yaml
#	- Kalman variance: 		class KF
*
* Outputs:
*   - Complete estimated state : \kalman_rocket_state
*
*/

#include "ros/ros.h"

#include "real_time_simulator/FSM.h"
#include "real_time_simulator/State.h"
#include "real_time_simulator/Control.h"
#include "real_time_simulator/Sensor.h"

#include "geometry_msgs/Vector3.h"

#include <time.h>
#include <sstream>
#include <string>

#include <iostream>
#include <fstream>
#include <chrono>
#include <iomanip>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include <unsupported/Eigen/EulerAngles>

#include "rocket_model.hpp"

#include <type_traits>


// Global variable with last received fsm
real_time_simulator::FSM current_fsm;

// Global variable with last received control
real_time_simulator::Control current_control;

// Global variable with last received sensor data
real_time_simulator::Sensor current_sensor;

// Global variable with last received real state from simulator
real_time_simulator::State current_rocket_state;

Rocket rocket;

std::ofstream myfile;

class KF {

	public:
		double Q;
		double R;

		double P;

		Eigen::Matrix<double, 14, 1> X;

		void init_KF(ros::NodeHandle n)
		{
			//Get initial orientation and convert in Radians
			float roll = 0, zenith = 0, azimuth = 0.0;
			n.getParam("/environment/rocket_roll", roll);
			n.getParam("/environment/rail_zenith", zenith);
			n.getParam("/environment/rail_azimuth", azimuth);

			roll *= 3.14159/180; zenith *= 3.14159/180; azimuth *= 3.14159/180;

			typedef Eigen::EulerSystem<-Eigen::EULER_Z, Eigen::EULER_Y, Eigen::EULER_Z> Rail_system;
			typedef Eigen::EulerAngles<double, Rail_system> angle_type;

			angle_type init_angle(azimuth, zenith, roll);

			Eigen::Quaterniond q(init_angle);

			// Init state X   
			X << 0, 0, 0,   0, 0, 0,     0.0, 0.0 , 0.0 , 1.0 ,     0.0, 0.0, 0.0,    rocket.propellant_mass;
			X.segment(6,4) = q.coeffs();

		
			// Initialize kalman parameters
			P = 0;

			Q = 1;

			R = 5;
		}
		
		void state_dynamics(Eigen::Ref<Eigen::Matrix<double, 14, 1>> x, Eigen::Ref<Eigen::Matrix<double, 14, 1>> xdot)
		{
			// -------------- Simulation variables -----------------------------
			double g0 = 9.81;  // Earth gravity in [m/s^2]

			Eigen::Matrix<double, 3, 1> rocket_control;
			rocket_control << current_control.force.x, current_control.force.y, current_control.force.z;

			// Orientation of the rocket with quaternion
			Eigen::Quaternion<double> attitude(x(9), x(6), x(7), x(8));
			attitude.normalize();
			Eigen::Matrix<double, 3, 3> rot_matrix = attitude.toRotationMatrix();

			// Current acceleration and angular rate from IMU
			Eigen::Matrix<double, 3, 1> IMU_acc; IMU_acc << current_sensor.IMU_acc.x, current_sensor.IMU_acc.y, current_sensor.IMU_acc.z;
			x.segment(10,3) << current_sensor.IMU_gyro.x, current_sensor.IMU_gyro.y, current_sensor.IMU_gyro.z;

			// Angular velocity omega in quaternion format to compute quaternion derivative
			Eigen::Quaternion<double> omega_quat(0.0, x(10), x(11), x(12));
			// -------------- Differential equation ---------------------

			// Position variation is speed
			xdot.head(3) = x.segment(3,3);

			// Speed variation is acceleration
			xdot.segment(3,3) =  rot_matrix*IMU_acc - Eigen::Vector3d::UnitZ()*g0;

			// Quaternion variation is 0.5*w◦q
			xdot.segment(6, 4) =  0.5*(omega_quat*attitude).coeffs();

			// Angular speed assumed to be constant between two measure
			xdot.segment(10, 3) << 0, 0, 0;

			// Mass variation is proportional to total thrust
			xdot(13) = -rocket_control.norm()/(rocket.Isp*g0);
		}

		void RK4(double dT)
		{
			Eigen::Matrix<double, 14, 1> k1, k2, k3, k4, X_inter;

			state_dynamics(X, k1); 			X_inter = X+k1*dT/2;
			state_dynamics(X_inter, k2); 	X_inter = X+k2*dT/2;
			state_dynamics(X_inter, k3); 	X_inter = X+k3*dT;
			state_dynamics(X_inter, k4);

			X = X + (k1+2*k2+2*k3+k4)*dT/6;
			 
		}

		void predict_step()
		{ 
			static double last_predict_time = ros::Time::now().toSec();

			double dT = ros::Time::now().toSec() - last_predict_time;
			RK4(dT);
			last_predict_time = ros::Time::now().toSec();

			P += Q;
		}

		void update_step(double z_baro)
		{	
			double K = P/(P+R);

			X(2) = X(2) + K*(z_baro - X(2));
			P = (1.0-K)*P;
		}
};
KF kalman;

// Callback function to store last received fsm
void fsmCallback(const real_time_simulator::FSM::ConstPtr& fsm)
{
	current_fsm.time_now = fsm->time_now;
  	current_fsm.state_machine = fsm->state_machine;
}

// Callback function to store last received control
void controlCallback(const real_time_simulator::Control::ConstPtr& control)
{
	current_control.torque = control->torque;
	current_control.force = control->force;
}

// Callback function to store last received state
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

	kalman.update_step(current_sensor.baro_height);
}


int main(int argc, char **argv)
{
	// Init ROS time keeper node
	ros::init(argc, argv, "data_fusion");
	ros::NodeHandle n;

	// Create filtered rocket state publisher
	ros::Publisher kalman_pub = n.advertise<real_time_simulator::State>("kalman_rocket_state", 10);

	// Subscribe to time_keeper for fsm and time
	ros::Subscriber fsm_sub = n.subscribe("gnc_fsm_pub", 100, fsmCallback);

	// Subscribe to control for kalman estimator
	ros::Subscriber control_sub = n.subscribe("control_pub", 100, controlCallback);

	// Subscribe to state message from simulation
  	ros::Subscriber rocket_state_sub = n.subscribe("rocket_state", 100, rocket_stateCallback);

	// Subscribe to sensor for kalman correction
	ros::Subscriber sensor_sub = n.subscribe("sensor_pub", 100, sensorCallback);

	// Initialize fsm
	current_fsm.time_now = 0;
	current_fsm.state_machine = "Idle";

	// Initialize rocket class with useful parameters
	rocket.init(n);

	// init EKF
	kalman.init_KF(n);

	// Thread to compute kalman. Duration defines interval time in seconds
	ros::Timer control_thread = n.createTimer(ros::Duration(0.010),
	[&](const ros::TimerEvent&) 
	{
		//double time_now = ros::Time::now().toSec();


		// State machine ------------------------------------------
		if (current_fsm.state_machine.compare("Idle") == 0)
		{
			// Do nothing
		}

		else if (current_fsm.state_machine.compare("Launch") == 0 || current_fsm.state_machine.compare("Rail") == 0)
		{
			kalman.predict_step();
		}

		else if (current_fsm.state_machine.compare("Coast") == 0)
		{
			kalman.predict_step();
		}
		// Parse kalman state and publish it on the /kalman_pub topic
		real_time_simulator::State kalman_state;

		kalman_state.pose.position.x = kalman.X(0);
		kalman_state.pose.position.y = kalman.X(1);
		kalman_state.pose.position.z = kalman.X(2);

		kalman_state.twist.linear.x = kalman.X(3);
		kalman_state.twist.linear.y = kalman.X(4);
		kalman_state.twist.linear.z = kalman.X(5);

		kalman_state.pose.orientation.x = kalman.X(6);
		kalman_state.pose.orientation.y = kalman.X(7);
		kalman_state.pose.orientation.z = kalman.X(8);
		kalman_state.pose.orientation.w = kalman.X(9);

		kalman_state.twist.angular.x = kalman.X(10);
		kalman_state.twist.angular.y = kalman.X(11);
		kalman_state.twist.angular.z = kalman.X(12);

		kalman_state.propeller_mass = kalman.X(13);


		kalman_pub.publish(kalman_state);
	});

	// Automatic callback of service and publisher from here
	ros::spin();

}
