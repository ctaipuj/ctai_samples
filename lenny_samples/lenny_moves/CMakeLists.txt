cmake_minimum_required(VERSION 2.8.3)
project(lenny_moves)

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(catkin REQUIRED
  COMPONENTS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
    pluginlib
    geometric_shapes
    robotiq_s_model_control
    motoman_driver
    gripper3f_control
    motoman_variables
    robotiq_85_control
)

find_package(Boost REQUIRED system filesystem date_time thread)

catkin_package(
  LIBRARIES
    interactivity_utils
    motoman_variables
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
  DEPENDS
    EIGEN3
  INCLUDE_DIRS include
)

###########
## Build ##
###########
include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(lenny_routine src/lenny_routine.cpp)
target_link_libraries(lenny_routine ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(lenny_joint_sample src/lenny_joint_move_sample.cpp)
target_link_libraries(lenny_joint_sample ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(lenny_pose_goal src/lenny_pose_goal.cpp)
target_link_libraries(lenny_pose_goal ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(lenny_write_variables src/lenny_write_to_25xxx_variables.cpp)
target_link_libraries(lenny_write_variables ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(lenny_attach_object src/lenny_attach_object.cpp)
target_link_libraries(lenny_attach_object ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(lenny_gripper_sample_node src/gripper_node_sample.cpp)
target_link_libraries(lenny_gripper_sample_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(sample_node src/sample_node.cpp)
target_link_libraries(sample_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
