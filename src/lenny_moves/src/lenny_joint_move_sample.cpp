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

//Modified by Nicolas---->this is a very dummy implementation to move the robot to four positions using the joints 

/*****************************************************
	THIS NODES REQUIRES ROSSERVICE robot_enable BE CALLED
	IS NOT RECOMMENDED CALLING THE SERVICE INSIDE A NODE. ROBOT COULD MOVE UNEXPECTEDLY!
	CAUTION WHEN CALLING robot_enable SERVICE MAKE SURE THE EMERGENCY STOP BUTTON IS REACHABLE BY OPERATOR
	****************************************************************/  

#include <moveit/move_group_interface/move_group_interface.h> //to move lenny
#include <moveit/planning_scene_interface/planning_scene_interface.h> //to move lenny

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial"); //dummy node starts
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  /* MoveIt! operates on sets of joints called "planning groups" and stores them in an object called
   the `JointModelGroup`. Throughout MoveIt! the terms "planning group" and "joint model group"
   are used interchangably.*/
  static const std::string PLANNING_GROUP = "arm_left"; //the group you want to move

  /*The :move_group_interface:`MoveGroup` class can be easily
  setup using just the name of the planning group you would like to control and plan for.*/
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP); //Update the current joint state
      
  move_group.setPlanningTime(20.0); //set allowed planning time
  move_group.setNumPlanningAttempts(8); //planning attempts
  //////////////////////
  move_group.setMaxVelocityScalingFactor(0.1); //TODO change speed IMPORTANT!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! 
  /////////////////////
  
  std::vector<double> move_group_joint_goal(7); //this vector stores the different joint positions according to the planning group order is important here
  
  bool success; //flag

  ROS_INFO("Reference frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO("End effector link: %s", move_group.getEndEffectorLink().c_str());
  
  ros::Rate rate(10.0);
  while (node_handle.ok()){ //while ok; this node will execute a routine 
   
  move_group_joint_goal[0]=3.110521078109741; //radians. You can fill the vector your preferred method.
  move_group_joint_goal[1]= -1.0633726119995117;
  move_group_joint_goal[2]= -0.24375106394290924;
  move_group_joint_goal[3]= 0.7988698482513428;
  move_group_joint_goal[4]= 1.470738172531128;
  move_group_joint_goal[5]= -1.6382914781570435;
  move_group_joint_goal[6]= 0.1707230508327484; //for the 7 joints a position. you can get the position using /joint_states
  
  move_group.setJointValueTarget(move_group_joint_goal); //update target

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan; //create a plan

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS); //plan trajectory

  ROS_INFO("Trajectory planning 1 %s", success ? "SUCCEED" : "FAILED");
  if(success)
  move_group.move(); //MOVES THE ROBOT 
  
  joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP); //update robot current state
   
  move_group_joint_goal[0]=2.52677321434021;
  move_group_joint_goal[1]= -0.9702940583229065;
  move_group_joint_goal[2]= 0.17251966893672943;
  move_group_joint_goal[3]= 1.2524117231369019;
  move_group_joint_goal[4]= 2.000751256942749;
  move_group_joint_goal[5]= -1.8992503881454468;
  move_group_joint_goal[6]= -2.4615280628204346;//new joint state
  
  move_group.setJointValueTarget(move_group_joint_goal); //set as target

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS); //plan

  ROS_INFO("Trajectory planning 2 %s", success ? "SUCCEED" : "FAILED");
  if(success)
  move_group.move(); //MOVES THE ROBOT 
  
  joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP); //update  
  
  move_group_joint_goal[0]= 0.05057023465633392;
  move_group_joint_goal[1]= 0.8450871109962463;
  move_group_joint_goal[2]= 1.2290983200073242;
  move_group_joint_goal[3]= -0.1578177660703659;
  move_group_joint_goal[4]= 3.0160491466522217;//new joint state
  move_group_joint_goal[5]= 1.0708097219467163;
  move_group_joint_goal[6]= -2.5531158447265625;
  
  move_group.setJointValueTarget(move_group_joint_goal); //set as target
   
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS); //plan

  ROS_INFO("Trajectory planning 3 %s", success ? "SUCCEED" : "FAILED");
  if(success)
  move_group.move(); //MOVES THE ROBOT 
  
  joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP); //update
      
  move_group_joint_goal[0]=-8.522115967934951e-05;
  move_group_joint_goal[1]= 0.0001874865556601435;
  move_group_joint_goal[2]= 6.075171404518187e-05;
  move_group_joint_goal[3]= 0.00024300685618072748;
  move_group_joint_goal[4]= -0.00012150342809036374;//new joint state
  move_group_joint_goal[5]= 3.0375857022590935e-05;
  move_group_joint_goal[6]= 0.00012031223013764247;
  
  move_group.setJointValueTarget(move_group_joint_goal); //set as target
    
  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS); //plan

  ROS_INFO("Trajectory planning 5 %s", success ? "SUCCEED" : "FAILED");
  if(success)
  move_group.move(); //MOVES THE ROBOT 
  
  joint_model_group =move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  }
    
  ros::shutdown(); //OFF
  return 0;
}
