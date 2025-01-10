#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include "ur_cvsa/UR_control.h"

int main(int argc, char **argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "UR_control_node");
    ros::NodeHandle nh;

    // Start a ROS spinner
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Set up MoveIt interfaces
    moveit::planning_interface::MoveGroupInterface move_group("manipulator");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    /* / Set up the visual tools for MoveIt! (Optional: for visualization)
    moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();
    */

    UR_control ur;
    ur.configure();
    ur.run();

    ros::shutdown();
    return 0;
}
