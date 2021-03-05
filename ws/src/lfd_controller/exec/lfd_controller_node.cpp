#include <ros/ros.h>
#include "lfd_controller/lfd_controller_ros.hpp"


int main(int argc, char** argv) {

        ros::init(argc, argv, "LDF_Controller");
        ros::NodeHandle nh;

        ROS_INFO("Starting LDF Controller Node");

        LFDControllerROS ldf_node;

        if(!ldf_node.Initialize(nh)) {
                ROS_ERROR("%s: Failed to initialize cis supervisor.",
                                ros::this_node::getName().c_str());
                return EXIT_FAILURE;
        }

        ros::MultiThreadedSpinner spinner(3);
        spinner.spin();

        return EXIT_SUCCESS;
}
