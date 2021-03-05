/*  Copyright (C) 2020, Tzanis Anevlavis, Luigi Pannocchi.

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


///////////////////////////////////////////////////////////////////////////////
//
//  LFD Controller ROS wrapper
//
///////////////////////////////////////////////////////////////////////////////

#pragma once

#include <ros/ros.h>
#include <string>
#include <unordered_map>
#include <thread>

// Messages
#include <testbed_msgs/CustOdometryStamped.h>
#include <testbed_msgs/ControlSetpoint.h>

#include "lfd_controller/lfd_controller.hpp"
#include "utilities/network_parser/network_parser.hpp"


// =================================================================
// CLASS
//
class LFDControllerROS {
	public:
		LFDControllerROS();
		~LFDControllerROS();

		// Initialize this class by reading parameters and loading callbacks.
		bool Initialize(const ros::NodeHandle& n);

	private:
		ros::NodeHandle node_;
		std::string node_name_;

		// Initialized flag and name.
		bool received_reference_;
		bool active_;

		double old_t_;

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

		double thrust;

		// Callback on Pose 
		void onNewState( const testbed_msgs::CustOdometryStamped::ConstPtr& msg);

		// Callback on Setpoint
		void onNewSetpoint(const testbed_msgs::ControlSetpoint::ConstPtr& msg);

		// Publishers and subscribers.
		// Output publishers:
		ros::Publisher lfd_controller_ctrl_;
		ros::Publisher performance_pub_;

		// Input Topics names
		std::string state_topic_;
		std::string setpoint_topic_;

		// Output Topics names
		std::string ctrls_topic_;
		std::string performance_topic_;

		ros::Subscriber setpoint_sub_;
		ros::Subscriber state_sub_;


		// DATA -------------------------------------------------------
		std::array<double, stateSize1D> ctrl_gains_;
		double yaw_ctrl_gain_;

		// THREAD OBJECTS 
		std::thread net_disc_thr_;

		// ===========================================================
		// CLASSES
		LFDController* lfd_controller_;

		Eigen::Quaterniond quat_;
};
