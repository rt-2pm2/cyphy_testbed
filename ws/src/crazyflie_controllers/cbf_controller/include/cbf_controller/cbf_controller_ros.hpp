/*  Copyright (C) 2021, Luigi Pannocchi.
	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.
	This program is distributed in the hope that it will be useful, but
	WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
	See the GNU General Public License for more details.
	You should have received a copy of the GNU General Public License
	along with this program. If not, see <http://www.gnu.org/licenses/>.  */


////////////////////////////////////////////////////////////////////////
//
//  CBF Controller ROS wrapper
//
////////////////////////////////////////////////////////////////////////

#pragma once

#include <ros/ros.h>
#include <string>
#include <unordered_map>
#include <thread>

// Messages
#include <testbed_msgs/CustOdometryStamped.h>
#include <testbed_msgs/ControlSetpoint.h>

#include <testbed_msgs/ControlStamped.h>
#include "cbf_controller/cbf_controller.hpp"

/*
struct Thread_arg {
	double period;
	CISSupervisor* psupervisor;
	ros::Publisher ctrl_pub;
	ros::Publisher perf_pub;
	Eigen::Quaterniond quat;
};
*/

// =================================================================
// CLASS
//
class CBFControllerROS {

	public:
		CBFControllerROS();
		~CBFControllerROS();

		// Initialize this class by reading parameters and loading callbacks.
		bool Initialize(const ros::NodeHandle& n);

	private:
		ros::NodeHandle node_;
		std::string node_name_;

		// Initialized flag and name.
		bool received_reference_;
		bool isControlActive_;

		bool initialized_;

		std::string setpoint_type_;

		// Name of the area where the controller is located 
		std::string area_name_;
		std::string controller_name_;

		// Name of the vehicle controlled
		std::string vehicle_name_;

		// Load parameters and register callbacks.
		bool LoadParameters(const ros::NodeHandle& n);
		bool RegisterCallbacks(const ros::NodeHandle& n);
		bool SetUpPublications(const ros::NodeHandle& n);

		// Remember last time we got a state callback.
		double last_state_time_;
		ros::Time last_sent_time;

		testbed_msgs::ControlStamped control_msg_;

		// Callback on Pose 
		void onNewState(const testbed_msgs::CustOdometryStamped::ConstPtr& msg);
		// Callback on Setpoint
		void onNewSetpoint(const testbed_msgs::ControlSetpoint::ConstPtr& msg);

		// Publishers and subscribers.
		// Output publishers:
		ros::Publisher control_pub_;

		// Input Topics names
		std::string state_topic_;
		std::string setpoint_topic_;

		// Output Topics names
		std::string ctrls_topic_;

		ros::Subscriber setpoint_sub_;
		ros::Subscriber state_sub_;

		// DATA ------------------------------------------------
		// Gains
		double Kpos_;
		double Kvel_;
		double Kx_;
		double eta_;
		double Kr_;
		double delta_x_;
		double yaw_ctrl_gain_;

		// Vehicle Mass
		double Mass_;

		// THREAD OBJECTS 
		//Thread_arg periodic_thread_arg_;
		//std::thread periodic_thread_;

		// =====================================================
		// CLASSES
		CBFController* controller_;
};