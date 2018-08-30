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

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <gripper3f_control/gripper3f_control.h> //gripper control


int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1); //node 
  spinner.start();
  
  bool success; //flag
  moveit::planning_interface::MoveGroupInterface::Plan my_plan; //create a plan
  static const std::string PLANNING_GROUP = "arm_right"; 
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface; //create a planning scene
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP); //update joint states
  
  std::vector<double> move_group_joint_goal(7); //this vector stores the different joint positions according to the planning group order is important here
  move_group.setMaxVelocityScalingFactor(0.08);
      
  Gripper3f right_gripper; //Create the gripper in the node
  right_gripper.init(); //init routine
  right_gripper.setMode(1); //pinch mode on
  right_gripper.setSpeed(250); //speed
  right_gripper.setForce(150); //force

  move_group_joint_goal[0]= 0.7210089288455954; //radians. You can fill the vector your preferred method.
  move_group_joint_goal[1]= 0.5586092271658862;
  move_group_joint_goal[2]= -2.4699952666748364;
  move_group_joint_goal[3]= -1.0389738305230167;
  move_group_joint_goal[4]= 1.1182004550866163;
  move_group_joint_goal[5]= 1.4468082890868876;
  move_group_joint_goal[6]= -0.8765700059563049; //for the 7 joints a position. you can get the position using /joint_states
  
  move_group.setJointValueTarget(move_group_joint_goal); 

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS); //verify
  ROS_INFO("Visualizing plan: %s", success ? "SUCCEED" : "FAILED");
  
  move_group.move(); //move robot
  joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP); //update joint state
  right_gripper.moveto(75); //close gripper to 75
  
    // Adding/Removing Objects and Attaching/Detaching Objects
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Define a collision object ROS message.
  moveit_msgs::CollisionObject collision_object; //create the collition object
  collision_object.header.frame_id = "palm"; //"father frame"

  // The id of the object is used to identify it.
  collision_object.id = "bottle"; //id

  // Define a cylinder to add to the world.
  shape_msgs::SolidPrimitive primitive; //creating the shape_msgs
  primitive.type = primitive.CYLINDER; //BOX=1, SPHERE=2 CYLINDER=3 CONE=4 
  /*
  type uint8 
  	BOX_x 0
  	BOX_Y 1
  	BOX_Z 2
  	CYLINDER_HEIGHT 0
	CYLINDER_RADIUS 1
	SPHERE_RADIUS 0
	CONE_HEIGHT 0 
	CONE_RADIUS 1 */
  
  primitive.dimensions.resize(2); //vector dimensions according to items above
  primitive.dimensions[0] = 0.2;
  primitive.dimensions[1] = 0.02;

  // Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose cylinder_pose;
  cylinder_pose.orientation.w = 1.0;
  cylinder_pose.position.x = 0.0;
  cylinder_pose.position.y = 0.1;//wrt "father frame"
  cylinder_pose.position.z = 0.0;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(cylinder_pose);
  collision_object.operation = collision_object.ADD; //add object to collitions 

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object); //you can insert different objects using a vector of collition objects

  planning_scene_interface.addCollisionObjects(collision_objects); //add objects to planning interface
  
  // Now when we plan a trajectory it will avoid the obstacle
  ROS_INFO("Attach the object to the robot");
  move_group.attachObject(collision_object.id);
  
  move_group_joint_goal[0]= 1.939940333366394; //radians. You can fill the vector your preferred method.
  move_group_joint_goal[1]= 0.46583589911460876; 
  move_group_joint_goal[2]= -2.72699236869812;
  move_group_joint_goal[3]= -0.9303820729255676;  
  move_group_joint_goal[4]= 0.7509974837303162; 
  move_group_joint_goal[5]= 1.7884544134140015;
  move_group_joint_goal[6]= -0.195717915892601; //for the 7 joints a position. you can get the position using /joint_states
  
  move_group.setJointValueTarget(move_group_joint_goal);

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Visualizing plan: %s", success ? "SUCCEED" : "FAILED");
  
  move_group.move();
  joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  
  right_gripper.open(); //open gripper

  // Now, let's detach the collision object from the robot.
  ROS_INFO_NAMED("tutorial", "Detach the object from the robot");
  move_group.detachObject(collision_object.id); //place object
  
  move_group_joint_goal[0]= 0;   //radians. You can fill the vector your preferred method.
  move_group_joint_goal[1]= 0; 
  move_group_joint_goal[2]= 0; 
  move_group_joint_goal[3]= 0; 
  move_group_joint_goal[4]= 0; 
  move_group_joint_goal[5]= 0; 
  move_group_joint_goal[6]= 0;  //for the 7 joints a position. you can get the position using /joint_states
  
  move_group.setJointValueTarget(move_group_joint_goal);

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO("Visualizing plan: %s", success ? "SUCCEED" : "FAILED");
  
  move_group.move(); //move 
  joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  right_gripper.setMode(0); //set gripper to basic mode again

  //Now, let's remove the collision object from the world.
  ROS_INFO_NAMED("tutorial", "Remove the object from the world");
  std::vector<std::string> object_ids;
  object_ids.push_back(collision_object.id);
  planning_scene_interface.removeCollisionObjects(object_ids); //delete object from the world

  // END_TUTORIAL

  ros::shutdown();
  return 0;
}
