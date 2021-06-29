///////////////////////////////////////////////////////////////////////////////
//
// State aggregator, which fuses state information from different sources and 
// compose an overall estimate of the vehicle position. 
// 
//
///////////////////////////////////////////////////////////////////////////////

#ifndef STATE_AGGREGATOR_H
#define STATE_AGGREGATOR_H

#include <ros/ros.h>

#include <string>
#include <unordered_map>
#include <thread>
#include "Eigen/Dense"

#include <testbed_msgs/CustOdometryStamped.h>

#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <state_aggregator/ControlSensor.h>

#include "filter/filter.hpp"
#include "utilities/network_parser/network_parser.hpp"


using namespace Eigen;

// =================================================================
// CLASS
//
class StateAggregator {
	public:
		StateAggregator();
		~StateAggregator();

		// Initialize this class by reading parameters and loading callbacks.
		bool Initialize(const ros::NodeHandle& n);

	private:
		ros::NodeHandle node_;

		std::string area_name_;
		std::string target_name_;

		double _sigmax;
		double _sigmay;

		// Network Parser
		NetworkParser network_parser;

		// Load parameters and register callbacks.
		bool LoadParameters(const ros::NodeHandle& n);
		bool RegisterCallbacks();
		bool AssociateTopicsToCallbacks(const ros::NodeHandle& n);
		int UpdatePublishers();
		void net_discovery(int ms);

		// Remember last time we got a state callback.
		double last_state_time_;

		// Callback on Pose 
		void onNewPose(const boost::shared_ptr<geometry_msgs::PoseStamped const>& msg, void* arg);
		void onNewPosition( const boost::shared_ptr<geometry_msgs::PointStamped const>& msg, void* arg);

		// Services
		bool control_sensor(state_aggregator::ControlSensor::Request& req,
				state_aggregator::ControlSensor::Response& res);
		ros::ServiceServer sensor_service;

		// Publishers and subscribers.
		// COMM ------------------------------------------------------------
		// Output publishers and broadcaster:
		ros::Publisher ext_pos_pub_;
		ros::Publisher pose_pub_;
		ros::Publisher pose_rpy_pub_;
		ros::Publisher odometry_pub_;

		/**
		 * MAP containing sensor topic information
		 */
		std::unordered_map<std::string, TopicData> inchannels_; 

		/**
		 * MAP for active subscribers
		 */
		std::unordered_map<std::string, ros::Subscriber> active_subscriber; 

		std::string object_name_;
		// Topics names
		std::string vrpn_topic_;
		std::string gtrack_topic_;

		std::string ext_position_topic_;
		std::string ext_pose_topic_;
		std::string ext_att_rpy_topic_;
		std::string ext_odom_topic_;

		// Initialized flag and name.
		bool received_reference_;
		bool initialized_;
		std::string name_;


		// DATA ------------------------------------------------------------
		// Pubblication variables 
		geometry_msgs::PoseStamped ext_pose_msg_;
		geometry_msgs::Vector3Stamped ext_att_rpy_msg_;
		geometry_msgs::PointStamped ext_position_msg_;
		testbed_msgs::CustOdometryStamped ext_odometry_msg_;


		// FILTER
		Filter* _pfilt;

		// Thread for network discovery
		std::thread net_disc_thr;


		// ===========================================================
		// Helper variables
		Eigen::Vector3d p_;
		Eigen::Vector3d v_;
		Eigen::Vector3d a_;

		Eigen::Vector3d euler_;
		Eigen::Vector3d w_;

		Eigen::Quaterniond q_;
		Eigen::Quaterniond q_old_;
		Eigen::Quaterniond qd_;
		Eigen::Quaterniond q_pf_;
}; //\class StateAggregator


#endif
