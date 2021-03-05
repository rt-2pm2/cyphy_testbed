#include "Eigen/Dense"
#include "math.h"
#include <chrono>

#include "lfd_controller/lfd_controller_ros.hpp"
#include "lfd_controller/PerformanceMsg.h"

#include <testbed_msgs/ControlStamped.h>

#include "utilities/timeutils/timeutils.hpp"
#include "utilities/custom_conversion/custom_conversion.hpp"
#include <mutex>

void thread_fnc(void* p);
std::mutex mx;

// =================================================================
// CLASS
//
LFDControllerROS::LFDControllerROS():
	active_(false),
	setpoint_type_("stop"),
	initialized_(false) {
		old_t_ = ros::Time::now().toSec();
	}

LFDControllerROS::~LFDControllerROS() {};

bool LFDControllerROS::Initialize(const ros::NodeHandle& n) {
	node_ = n;
	ros::NodeHandle nl(n);

	// Set the node name
	node_name_ = ros::this_node::getName().c_str();

	// Load parameters
	if (!LoadParameters(nl)) {
		ROS_ERROR("%s: Failed to load parameters.", node_name_.c_str());
		return false;
	}

	// Register callbacks
	if (!RegisterCallbacks(n)) {
		ROS_ERROR("%s: Failed to register callbacks.", node_name_.c_str());
		return false;
	}

	// Instantiate classes
	lfd_controller_ = new LFDController();
	lfd_controller_->LoadK();

	lfd_controller_->SetKyaw(yaw_ctrl_gain_);

	// Setup output publications and services
	SetUpPublications(nl);

	initialized_ = true;

	thrust = 0.032 * 9.81;

	ROS_INFO("[%s] Initialized!", node_name_.c_str());
	return true;
}

bool LFDControllerROS::LoadParameters(const ros::NodeHandle& n) {

	ros::NodeHandle np("~");
	std::string key;

	// Controller name
	np.param<std::string>("param/controller_name", controller_name_, "LFDController");

	np.param<std::string>("param/area_name", area_name_, "area0");

	// Vehicle name
	np.param<std::string>("param/vehicle_name", vehicle_name_, "cf2");

	// Vehicle State 
	np.param<std::string>("topics/in_state_topic", state_topic_,
			"/" + vehicle_name_ + "/external_codom");

	// Vehicle Setpoint
	np.param<std::string>("topics/in_setpoint_topic", setpoint_topic_,
			"/" + vehicle_name_ + "/setpoint");

	// Control Cmd 
	np.param<std::string>("topics/out_ctrl_topic", ctrls_topic_ ,
			"/" + area_name_ + "/controller/" + controller_name_ + "/" +
			vehicle_name_ + "/control");
	// Performance Topic
	np.param<std::string>("topics/out_perf_topic", performance_topic_,
			"/" + controller_name_ + "/" + vehicle_name_ + "/cis_perf");

	std::string param_name;
	if (np.searchParam("param/ctrl_gains", param_name)) {
		std::cout << "Found " << param_name << std::endl; 
		std::vector<double> p_vec;
		n.getParam(param_name, p_vec);
		std::copy_n(p_vec.begin(), p_vec.size(), ctrl_gains_.begin());

		std::cout << "K_gain: ";
		for (auto el : ctrl_gains_) {
			std::cout << el << " ";
		}
		std::cout << std::endl;
	} else {
		ROS_INFO("No param 'param/ctrl_gains' found in an upward search");
	}

	if (np.searchParam("param/yaw_ctrl_gain", param_name)) {
		std::cout << "Found " << param_name << std::endl; 
		n.getParam(param_name, yaw_ctrl_gain_);
		std::cout << "Yaw_K_gain: ";
		std::cout << yaw_ctrl_gain_ << std::endl;
	} else {
		ROS_INFO("No param 'param/yaw_ctrl_gain' found in an upward search");
	}

	return true;
}

bool LFDControllerROS::SetUpPublications(const ros::NodeHandle& n) {

	ros::NodeHandle nl(n);

	// Output Publications
	// Control Signals (Thrust + Angular Velocities)
	lfd_controller_ctrl_ = nl.advertise<testbed_msgs::ControlStamped> (ctrls_topic_.c_str(), 5);

	// Eventual Performance Data
	performance_pub_ = nl.advertise<lfd_controller::PerformanceMsg> (performance_topic_.c_str(), 5);

	return true;
}


bool LFDControllerROS::RegisterCallbacks(const ros::NodeHandle& n) {
	ros::NodeHandle nl(n);
	setpoint_sub_ = nl.subscribe(setpoint_topic_.c_str(), 1,
			&LFDControllerROS::onNewSetpoint, this);
	state_sub_ = nl.subscribe(state_topic_.c_str(), 1,
			&LFDControllerROS::onNewState, this);
	return true;
}


// CALLBACKS ----------------------------------------------------------------
void LFDControllerROS::onNewState(
		const testbed_msgs::CustOdometryStamped::ConstPtr& msg) {
	// Take the time
	//ros::Time current_time = ros::Time::now();
	// Read the timestamp of the message
	//t.tv_sec = msg->header.stamp.sec;
	//t.tv_nsec = msg->header.stamp.nsec;
	XType state_;
	state_(0) = msg->p.x;
	state_(1) = msg->p.y;
	state_(2) = msg->p.z;
	state_(3) = msg->v.x;
	state_(4) = msg->v.y;
	state_(5) = msg->v.z;
	state_(6) = msg->a.x;
	state_(7) = msg->a.y;
	state_(8) = msg->a.z;

	lfd_controller_->SetState(state_);
	quat_.vec() = Eigen::Vector3d (msg->q.x, msg->q.y, msg->q.z);
	quat_.w() = msg->q.w;
	quat_.normalize();

	lfd_controller_->SetQuat(quat_);

	double t = msg->header.stamp.toSec();
	double dt = t - old_t_;
	old_t_ = t;
		
	UType control_cmd;
	UType u_body(UType::Zero()); 

	testbed_msgs::ControlStamped control_msg;
	lfd_controller::PerformanceMsg ctrl_perf_msg;

	if (lfd_controller_->isControlActive()) {
		bool active = lfd_controller_->Step(t);
		// Get the desired jerk
		control_cmd = lfd_controller_->getControls();

		// ... convert the jerk in autopilot commands
		// 1) Convert the jerk in body frame
		u_body = quat_.inverse() * control_cmd;
		// 2) Convert in angular velocity and thrust
		if (thrust > 0.05) {
			control_msg.control.roll = -(u_body(1) / thrust) * 0.032;
			control_msg.control.pitch = (u_body(0) / thrust) * 0.032;
		}
		thrust = std::max(thrust + 0.032 * u_body(2) * dt, 0.0);
		control_msg.control.thrust = thrust / 0.032; // Because the library works with acc
	} else {
		control_msg.control.thrust = 0.0;
		control_msg.control.roll = 0.0;
		control_msg.control.pitch = 0.0;
		control_msg.control.yaw_dot = 0.0;
	}

	control_msg.control.yaw_dot = lfd_controller_->getYawCtrl();

	// Crazyflie Fuck
	control_msg.control.pitch *= -1;

	// Performance Message
	ros::Time msg_timestamp = ros::Time::now();

	control_msg.header.stamp = msg_timestamp;
	ctrl_perf_msg.header.stamp = msg_timestamp;

	ctrl_perf_msg.thrust = thrust;
	for (int i = 0; i < outputSize; i++) {
		ctrl_perf_msg.jerk_body[i] = u_body(i);
		ctrl_perf_msg.jerk_world[i] = control_cmd(i);
	}
	ctrl_perf_msg.ang_velocity[0] = control_msg.control.roll;
	ctrl_perf_msg.ang_velocity[1] = control_msg.control.pitch;

	for (int i = 0; i < stateSize; i++) {
		ctrl_perf_msg.state_curr[i] = state_(i); 
	}
	ctrl_perf_msg.yaw_rate = control_msg.control.yaw_dot;  

	lfd_controller_ctrl_.publish(control_msg);
	performance_pub_.publish(ctrl_perf_msg);

	return;
}


// Process an incoming setpoint point change.
void LFDControllerROS::onNewSetpoint(
		const testbed_msgs::ControlSetpoint::ConstPtr& msg) {

	XType ctrl_setpoint;
	UType ctrl_ff;

	if (msg->setpoint_type != "stop") {
		active_ = true;
		ctrl_setpoint(0) = msg->p.x;
		ctrl_setpoint(1) = msg->p.y;
		ctrl_setpoint(2) = msg->p.z;
		ctrl_setpoint(3) = msg->v.x;
		ctrl_setpoint(4) = msg->v.y;
		ctrl_setpoint(5) = msg->v.z;
		ctrl_setpoint(6) = msg->a.x;
		ctrl_setpoint(7) = msg->a.y;
		ctrl_setpoint(8) = msg->a.z;

		ctrl_ff(0) = msg->j.x;
		ctrl_ff(1) = msg->j.y;
		ctrl_ff(2) = msg->j.z;

		// Copy the setpoint structure into the controller class
		lfd_controller_->SetActive(true);
		lfd_controller_->SetSetpoint(ctrl_setpoint);
		lfd_controller_->SetFeedForward(ctrl_ff);

		// Set the flag about the setpoint
	} else {
		lfd_controller_->SetActive(false);
		active_ = false;
	}
}
