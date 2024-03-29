#ifndef SIMULATOR_HPP
#define SIMULATOR_HPP

#include <ros/ros.h>
#include "geometry_msgs/Vector3Stamped.h"
#include "testbed_msgs/ControlStamped.h"
#include <thread>
#include "Eigen/Dense"

#include "simclass/dyn_class.hpp"
#include "timespec_lib/timespec_lib.hpp"

struct simThread_arg {
    double period;
    void* pParam;
    IDynamics* pSim;
    ros::Publisher pub;
};

class XSimulator {
    public:
        XSimulator();

        bool Initialize(const ros::NodeHandle& n);
        bool LoadParameters(const ros::NodeHandle& n);
        bool RegisterCallbacks(const ros::NodeHandle& n);

        // List of Callbacks methods
        void ControlCallback(
                const testbed_msgs::ControlStamped::ConstPtr& msg);

        void pub_thread_fnc(double dt);
	void start_simulation(double dt);
	void start_sensor_pub(double dt);

    private:
        std::string name_;
        std::string frame_name_;

        ros::Time old_time_;

	std::string actuation_;
	IDynamics* sim_;

        SimParams parameters_;
        simThread_arg arg_;

	double noise_std_;

        bool initialized_;

        ros::Subscriber ctrl_topic_sub_;
        ros::Publisher state_pub_;
        ros::Publisher vrpn_sim_pub_;

        std::string sim_sensor_topic_;
        std::string sim_state_topic_;
        std::string ctrl_topic_; 
        std::string vrpn_sim_pose_topic_;

	std::vector<double> initial_pos_;

        double sim_period_;
	double sens_period_;
        double Mass_;
        double c_drag_;
        double a_drag_;

        void updateState(double dt);

	double generate_noise();

        std::thread sim_thread;
        std::thread pub_thread;
};

#endif
