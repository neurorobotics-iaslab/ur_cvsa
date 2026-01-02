#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include "ur_bci/UR_control.h"

int main(int argc, char **argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "UR_control_node");
    ros::NodeHandle nh;

    // Start a ROS spinner
    ros::AsyncSpinner spinner(1);
    spinner.start();

    UR_control ur;
    ur.configure();
    ur.run();

    ros::shutdown();
    return 0;
}
