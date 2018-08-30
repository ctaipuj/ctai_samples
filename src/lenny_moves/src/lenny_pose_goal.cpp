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

#include <moveit/move_group_interface/move_group_interface.h> //to move lenny
#include <moveit/planning_scene_interface/planning_scene_interface.h> //to move lenny

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial"); //dummy node starts
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
  move_group.setGoalPositionTolerance(0.008);
  move_group.setGoalOrientationTolerance(0.1);
  move_group.setMaxVelocityScalingFactor(0.2); //////TODO set speed before execute the node!
  
  bool success;

  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the
  // end-effector.
  geometry_msgs::Pose target_pose; //define a pose from geometry msgs.

  target_pose.orientation.w = 0.010;
  target_pose.orientation.x = -0.010;
  target_pose.orientation.y = -0.022;
  target_pose.orientation.z = 1;
  
  target_pose.position.x = 0.159; //0.7
  target_pose.position.y = 0.792; //0.25
  target_pose.position.z = 1.152; //1.2
  
  move_group.setPoseTarget(target_pose,"arm_left_link_7_t");
  
  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO("Moving to planned goal pose %s", success ? "" : "FAILED");
  
  move_group.move();
  
  joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  
  std::vector<geometry_msgs::Pose> waypoints;

  geometry_msgs::Pose target_pose2 = target_pose;
  target_pose2.position.x += 0.07;
  waypoints.push_back(target_pose2);  // up and out
  
  target_pose2.position.z += 0.05;
  waypoints.push_back(target_pose2);

  target_pose2.position.y -= 0.05;
  waypoints.push_back(target_pose2);  // left
  
  /*target_pose2.position.z -= 0.05;
  target_pose2.position.y += 0.05;
  target_pose2.position.x -= 0.07;
  waypoints.push_back(target_pose2);*/
  
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double fraction=move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  
  ROS_INFO("Visualizing plan (cartesian path) (%.2f%% achieved)", fraction * 100.0);
  
  move_group.setMaxVelocityScalingFactor(0.1);
  
  //my_plan.trajectory_=trajectory;
  
  //move_group.execute(my_plan);
  
  move_group.move();
    
  ros::shutdown();
  return 0;
}
