#include "ur_cvsa/UR_control.h"

UR_control::UR_control() : move_group("manipulator"), nh_("~") {    
    this->sub_events_ = this->nh_.subscribe("/events/bus", 1, &UR_control::neuro_callback, this);

    this->srv_robot_motion_ = this->nh_.advertiseService("cvsa/robot_motion", &UR_control::robot_state, this);
}

bool UR_control::configure(){
    if(ros::param::get("~classes", this->classes_) == false){
        ROS_ERROR("[UR_control] Missing 'classes' parameter, which is a mandatory parameter");
        return false;
    }

    this->robot_is_moving_ = false;
    this->m_max_attempt_ = 10;

    // TODO: set the joints for the different classes, parameter this->position_.joints_classes e timeout

    return true;
}

void UR_control::run(){
    ros::Rate r(256);
    while(ros::ok()){
        ros::spinOnce();
        r.sleep();
    }
}

bool UR_control::robot_state(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    res.success = this->robot_is_moving_;
    return true;
}

bool UR_control::goJoint(std::vector<double> joints) {
    this->move_group.getCurrentJointValues();
    bool plan_success = false;
    bool exec_success = false;
    this->move_group.setJointValueTarget(joints);
    for(int i = 0; i < this->m_max_attempt_; i++){
        plan_success = (this->move_group.plan(this->my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if(plan_success){
            ros::Duration(0.2).sleep();
            break;
        }
        ros::Duration(0.5).sleep();
    }
    if(plan_success){
        exec_success = (this->move_group.execute(this->my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if(!exec_success){
            ROS_ERROR("[UR_control] Execution failed!");
            return false;
        }
    }else{
        ROS_ERROR("[UR_control] Planning failed!");
        return false;
    }

    return true;
}

bool UR_control::goPose(geometry_msgs::Pose pose) {
    this->move_group.getCurrentState();
    bool plan_success = false;
    bool exec_success = false;
    this->move_group.setPoseTarget(pose);
    for(int i = 0; i < this->m_max_attempt_; i++){
        plan_success = (this->move_group.plan(this->my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if(plan_success){
            ros::Duration(0.2).sleep();
            break;
        }
        ros::Duration(0.5).sleep();
    }
    if(plan_success){
        exec_success = (this->move_group.execute(this->my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if(!exec_success){
            ROS_ERROR("[UR_control] Execution failed!");
            return false;
        }
    }else{
        ROS_ERROR("[UR_control] Planning failed!");
        return false;
    }

    return true;
}

bool UR_control::goHome() {
    // intermediate position
    std::vector<double> joints = {-1.7246678511248987, -1.7996023336993616, 2.1641526222229004, -1.9401467482196253, -1.5369065443622034, -0.11355048814882451};
    if(!this->goJoint(joints)){
        ROS_ERROR("[UR_control] Error in moving to intermediate position");
        return false;
    }
    joints = {-1.9337423483477991, -2.231931511555807, 2.3062686920166016, -1.626599136983053, -1.5628321806537073, -0.3018081823932093};
    if(!this->goJoint(joints)){
        ROS_ERROR("[UR_control] Error in moving to home position");
        return false;
    }

    return true;
}

bool UR_control::goToObject(int boom){
    std::vector<double> joints;
    int idx_class;
    if(boom == event_.HIT){
        auto min_class = std::min_element(this->classes_.begin(), this->classes_.end());
        idx_class = this->cue_ - * min_class;
        joints = this->position_.joints_classes[idx_class];
        ROS_INFO("[UR_control] Robot moves to %d", this->classes_[idx_class]);
    }else if(boom == event_.MISS){
        std::vector<int> classes_copy = this->classes_;
        auto it = std::remove(classes_copy.begin(), classes_copy.end(), this->cue_);
        classes_copy.erase(it, classes_copy.end());
        if(classes_copy.empty()){
            ROS_ERROR("[UR_control] Error to select the class in which the robot moves");
        }else{
            // take one random of the other, if two classes then is the other
            std::srand(static_cast<unsigned int>(std::time(0)));
            int idx_rand = std::rand() % classes_copy.size();
            auto min_class = std::min_element(this->classes_.begin(), this->classes_.end());
            idx_class = classes_copy[idx_rand] - *min_class;
            joints = this->position_.joints_classes[idx_class];
            ROS_INFO("[UR_control] Robot moves to %d", this->classes_[idx_class]);
        }
        ROS_INFO("[UR_control] Robot moves to %d", this->classes_[idx_class]);
    }else if(boom == event_.TIMEOUT){
        joints = this->position_.joints_timeout;
        ROS_INFO("[UR_control] Robot moves to timeout position");
    }
    this->robot_is_moving_ = true;
    if(!this->goJoint(joints)){
        ROS_ERROR("[UR_control] Error in moving to home position");
        return false;
    }

    return true;
}

void UR_control::neuro_callback(const rosneuro_msgs::NeuroEvent &msg) {
    // check the target given by the cue
    auto it = std::find(this->classes_.begin(), this->classes_.end(), msg.event);
    if(it != this->classes_.end()){
        this->cue_ = msg.event;
    }else{
        // check if hit or miss or timeout
        int c_event = msg.event;
        switch (c_event){
            case event_.HIT:
                this->goToObject(event_.HIT);
                this->goHome();
                this->robot_is_moving_ = false;
                break;
            
            case event_.MISS:
                this->goToObject(event_.MISS);
                this->goHome();
                this->robot_is_moving_ = false;
                break;

            case event_.TIMEOUT:
                this->goToObject(event_.TIMEOUT);
                this->goHome();
                this->robot_is_moving_ = false;
                break;

            default:
                break;
        }

    }
}

UR_control::~UR_control() {
    ros::shutdown();
}

