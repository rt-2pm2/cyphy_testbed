#ifndef SIMULATOR_HPP
#define SIMULATOR_HPP

#include <ros/ros.h>
#include "geometry_msgs/Vector3Stamped.h"
#include "testbed_msgs/ControlStamped.h"
#include <thread>
#include "Eigen/Dense"

#include "simclass/simdyn.hpp"
#include "simulator/dyn_model.hpp"
#include "timespec_lib/timespec_lib.hpp"

struct State {
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Quaterniond q;
};

struct simThread_arg {
    double period;
    void* pParam;
    SimDyn* pSim;
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

    private:
        State X;

        std::string name_;
        std::string frame_name_;

        ros::Time old_time_;

        SimDyn* sim_;
        SimParam parameters_;
        simThread_arg arg_;

        bool initialized_;

        ros::Subscriber ctrl_topic_sub_;
        ros::Publisher codometry_pub_;
        ros::Publisher sensor_pub_;
        ros::Publisher vrpn_sim_pub_;

        std::string sim_sensor_topic_;
        std::string sim_state_topic_;
        std::string ctrl_topic_; 
        std::string vrpn_sim_pose_topic_;

        double sim_period_;
        double Mass_;
        double c_drag_;
        double a_drag_;

        void updateState(double dt);

        std::thread sim_thread;
        std::thread pub_thread;
};

#endif
