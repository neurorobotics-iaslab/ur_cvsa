#ifndef UR_CONTROL_H_
#define UR_CONTROL_H_

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rosneuro_msgs/NeuroEvent.h>
#include <algorithm>
#include <std_srvs/Trigger.h>
#include <ctime>
#include <cstdlib>


struct Event{
    const int HIT = 897;
    const int MISS = 898;
    const int TIMEOUT = 899;
};
constexpr Event event_;

struct Position_joints{
    std::vector<std::vector<double>> joints_classes = {{1.6201391220092773, -1.1530116240130823, -0.8488729635821741, -2.0379536787616175, -1.5707748571978968, -0.8482573668109339},
                                                       {1.678596019744873, -1.1926682631122034, -2.6249547640429896, -2.056678120289938, -1.5708468596087855, -2.6241264978991907}}; // place here the positions
    std::vector<double> joints_timeout = {1.4639124870300293, -1.1009696165667933, -1.5708625952350062, -1.9338977972613733, -1.570798699055807, -1.5702832380877894};
};


class UR_control{
    public:
        UR_control();
        ~UR_control();
        void neuro_callback(const rosneuro_msgs::NeuroEvent &msg);
        bool configure();
        bool robot_state(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
        bool goJoint(std::vector<double> joints);
        bool goPose(geometry_msgs::Pose pose);
        bool goHome();
        bool goToObject(int boom);
        void run();

    private:
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        moveit::planning_interface::MoveGroupInterface move_group;
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        ros::NodeHandle nh_;
        ros::Subscriber sub_events_;
        std::vector<int> classes_;
        std::vector<int> boom_results_;
        int cue_;
        ros::ServiceServer srv_robot_motion_;
        bool robot_is_moving_;
        int m_max_attempt_;

        Position_joints position_;
};


#endif