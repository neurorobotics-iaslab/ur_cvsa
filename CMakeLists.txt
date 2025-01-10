cmake_minimum_required(VERSION 3.0.2)
project(ur_cvsa)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  moveit_core
  moveit_ros_planning_interface
  std_srvs
  roscpp
)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    ${PROJECT_NAME}
  CATKIN_DEPENDS 
    geometry_msgs 
    moveit_core 
    moveit_ros_planning_interface 
    roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} 
              src/UR_control.cpp
            )
target_link_libraries(${PROJECT_NAME} 
            ${catkin_LIBRARIES}
)

add_executable(ur_cvsa_node src/UR_control_node.cpp)
target_link_libraries(ur_cvsa_node 
                          ${catkin_LIBRARIES} 
                          ${PROJECT_NAME}
                      )  
