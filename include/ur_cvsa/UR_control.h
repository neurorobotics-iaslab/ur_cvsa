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
    std::vector<std::vector<double>> joints_classes = {{-1.0177915732013147, -1.0552557150470179, 1.5992155075073242, -2.1148069540606897, -1.571134392415182, -1.0169943014727991},
                                                       {-2.443615261708395, -1.0814431349383753, 1.641397476196289, -2.1309402624713343, -1.5708826223956507, -2.442632500325338}}; // place here the positions
    std::vector<double> joints_timeout = {-1.570719067250387, -1.3460143248187464, 2.055833339691162, -2.2806666533099573, -1.5707867781268519, -1.5698874632464808};
    std::vector<double> joints_home = {-1.570793628692627, -1.5709403196917933, 1.5708029905902308, -1.570796314870016, -1.5709064642535608, -1.5700791517840784};
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
        const moveit::core::JointModelGroup* joint_model_group_;
        moveit::planning_interface::MoveGroupInterface move_group_;
        moveit::planning_interface::MoveGroupInterface::Plan my_plan_;

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