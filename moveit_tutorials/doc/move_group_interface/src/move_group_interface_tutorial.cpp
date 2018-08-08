/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of SRI International nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // BEGIN_TUTORIAL
  //
  // Setup
  // ^^^^^
  //
  // MoveIt! operates on sets of joints called "planning groups" and stores them in an object called
  // the `JointModelGroup`. Throughout MoveIt! the terms "planning group" and "joint model group"
  // are used interchangably.
  static const std::string PLANNING_GROUP = "arm_left";

  // The :move_group_interface:`MoveGroup` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
      
  move_group.setPlanningTime(20.0);
  move_group.setNumPlanningAttempts(8);
  
  std::vector<double> move_group_joint_goal(7);

  // Visualization
  // ^^^^^^^^^^^^^
  //
  // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
  visual_tools.deleteAllMarkers();

  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();

  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  // Start the demo
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the
  // end-effector.
  /*geometry_msgs::Pose target_pose1;

  target_pose1.orientation.w = 0.058;
  target_pose1.orientation.x = -0.060;
  target_pose1.orientation.y = 0.046;
  target_pose1.orientation.z = 0.995;
  
  target_pose1.position.x = 0.849; //0.7
  target_pose1.position.y = 0.318; //0.25
  target_pose1.position.z = 1.1; //1.2
  move_group.setPoseTarget(target_pose1);*/
  while(1){
  
  move_group_joint_goal[0]=3.110521078109741;
  move_group_joint_goal[1]= -1.0633726119995117;
  move_group_joint_goal[2]= -0.24375106394290924;
  move_group_joint_goal[3]= 0.7988698482513428;
  move_group_joint_goal[4]= 1.470738172531128;
  move_group_joint_goal[5]= -1.6382914781570435;
  move_group_joint_goal[6]= 0.1707230508327484;
  
  move_group.setJointValueTarget(move_group_joint_goal);

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // Visualizing plans
  // ^^^^^^^^^^^^^^^^^
  // We can also visualize the plan as a line with markers in RViz.
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  //visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // Moving to a pose goal
  // ^^^^^^^^^^^^^^^^^^^^^
  //
  // Moving to a pose goal is similar to the step above
  // except we now use the move() function. Note that
  // the pose goal we had set earlier is still active
  // and so the robot will try to move to that goal. We will
  // not use that function in this tutorial since it is
  // a blocking function and requires a controller to be active
  // and report success on execution of a trajectory.

  /* Uncomment below line when working with a real robot */
    move_group.setMaxVelocityScalingFactor(0.6);
    move_group.move();

  // Planning to a joint-space goal
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Let's set a joint space goal and move towards it.  This will replace the
  // pose target we set above.
  //
  // To start, we'll create an pointer that references the current robot's state.
  // RobotState is the object that contains all the current position/velocity/acceleration data.
  joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
      
  /*geometry_msgs::Pose target_pose2;
  target_pose2.orientation.w = 1.0;
  target_pose2.position.x = 0.7;
  target_pose2.position.y = 0.25;
  target_pose2.position.z = 1.3;
  move_group.setPoseTarget(target_pose2);*/
  
  move_group_joint_goal[0]=2.52677321434021;
  move_group_joint_goal[1]= -0.9702940583229065;
  move_group_joint_goal[2]= 0.17251966893672943;
  move_group_joint_goal[3]= 1.2524117231369019;
  move_group_joint_goal[4]= 2.000751256942749;
  move_group_joint_goal[5]= -1.8992503881454468;
  move_group_joint_goal[6]= -2.4615280628204346;
  
  move_group.setJointValueTarget(move_group_joint_goal);
  
  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan2;

  bool success2 = (move_group.plan(my_plan2) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (pose goal) %s", success2 ? "" : "FAILED");
  
  move_group.move();
  ////////////////////////
  joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
      
  /*geometry_msgs::Pose target_pose3;
  
  target_pose3.orientation.w = 0.610;
  target_pose3.orientation.x = 0.158;
  target_pose3.orientation.y = -0.081;
  target_pose3.orientation.z = -0.773;
  
  target_pose3.position.x = 0.788;
  target_pose3.position.y = 0.701;
  target_pose3.position.z = 0.985;
  move_group.setPoseTarget(target_pose3);*/
  
  move_group_joint_goal[0]= 0.05057023465633392;
  move_group_joint_goal[1]= 0.8450871109962463;
  move_group_joint_goal[2]= 1.2290983200073242;
  move_group_joint_goal[3]= -0.1578177660703659;
  move_group_joint_goal[4]= 3.0160491466522217;
  move_group_joint_goal[5]= 1.0708097219467163;
  move_group_joint_goal[6]= -2.5531158447265625;
  
  move_group.setJointValueTarget(move_group_joint_goal);
  
  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  
  moveit::planning_interface::MoveGroupInterface::Plan my_plan3;
  
  success = (move_group.plan(my_plan3) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (pose goal) %s", success ? "" : "FAILED");
  
  move_group.move();
  
  /////////////
  
  joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
      
  /*geometry_msgs::Pose target_pose4;
    
  target_pose4.orientation.w = 0.272;
  target_pose4.orientation.x = 0.138;
  target_pose4.orientation.y = 0.191;
  target_pose4.orientation.z = 0.933;
  
  target_pose4.position.x = -0.249;
  target_pose4.position.y = 0.732;
  target_pose4.position.z = 1.042;
  move_group.setPoseTarget(target_pose4);*/
  
  move_group_joint_goal[0]= -2.043586254119873;
  move_group_joint_goal[1]= 0.8177140951156616;
  move_group_joint_goal[2]= 1.141372799873352;
  move_group_joint_goal[3]= 1.0489087104797363;
  move_group_joint_goal[4]= 0.13020610809326172;
  move_group_joint_goal[5]= 1.2968060970306396;
  move_group_joint_goal[6]= -2.55305552482605;
  
  /*move_group_joint_goal[0]= -2.9048655033111572;
  move_group_joint_goal[1]= 1.111863374710083;
  move_group_joint_goal[2]= 1.2859010696411133;
  move_group_joint_goal[3]= 0.5115142464637756;
  move_group_joint_goal[4]= -0.2654849886894226;
  move_group_joint_goal[5]= -0.9050334692001343;
  move_group_joint_goal[6]= -0.33103910088539124;
  
  /*move_group_joint_goal[0]= -0.6949104070663452;
  move_group_joint_goal[1]= -0.7042165398597717;
  move_group_joint_goal[2]= -1.0528727769851685;
  move_group_joint_goal[3]= -0.7774244546890259;
  move_group_joint_goal[4]= 2.592958927154541;
  move_group_joint_goal[5]= -1.9000554084777832;
  move_group_joint_goal[6]= -2.921902894973755;*/
  
  move_group.setJointValueTarget(move_group_joint_goal);
  
  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  
  moveit::planning_interface::MoveGroupInterface::Plan my_plan4;
  
  success = (move_group.plan(my_plan4) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (pose goal) %s", success ? "" : "FAILED");
  
  move_group.move();
  
  ///////////////////////
  
  joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  
  move_group_joint_goal[0]=-8.522115967934951e-05;
  move_group_joint_goal[1]= 0.0001874865556601435;
  move_group_joint_goal[2]= 6.075171404518187e-05;
  move_group_joint_goal[3]= 0.00024300685618072748;
  move_group_joint_goal[4]= -0.00012150342809036374;
  move_group_joint_goal[5]= 3.0375857022590935e-05;
  move_group_joint_goal[6]= 0.00012031223013764247;
  
  move_group.setJointValueTarget(move_group_joint_goal);
  
  moveit::planning_interface::MoveGroupInterface::Plan my_plan5;
  
  success = (move_group.plan(my_plan5) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 5 (pose goal) %s", success ? "" : "FAILED");
  
  move_group.move();
  
  joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  }
  
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  //
  // Next get the current set of joint values for the group.
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
  
  ros::shutdown();
  return 0;
}
