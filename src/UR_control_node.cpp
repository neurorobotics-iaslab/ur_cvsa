#include <ur_cvsa/UR_control.h>
#include <ros/ros.h>


int main(int argc, char** argv) {

	// ros initialization
	ros::init(argc, argv, "ur_cvsa_node");

	UR_control ur_control;
	
	if(ur_control.configure() == false) {
		std::cerr<<"SETUP ERROR"<<std::endl;
		return -1;
	}

	ROS_INFO("[INFO] Configuration done");
	
	ur_control.run();
    
	ros::shutdown();
	return 0;
}
