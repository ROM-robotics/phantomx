#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char **argv)
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
  static const std::string PLANNING_GROUP = "arm";

  // The :move_group_interface:`MoveGroup` class can be easily
  // setup using just the name of the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to add and remove collision objects in our "virtual world" scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // Raw pointers are frequently used to refer to the planning group for improved performance.
  const robot_state::JointModelGroup *joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  
  
  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());
  
  //-----------------------------------------------------------------
  moveit_msgs::CollisionObject co;
  co.header.stamp = ros::Time::now();
  co.header.frame_id = "base_footprint"; // group.getPlanningFrame(); 
  co.id = "block";
  co.operation = moveit_msgs::CollisionObject::ADD;

  geometry_msgs::Pose pose; 
  pose.position.x = 0.31;
  pose.position.y = 0.01;
  pose.position.z = -0.01;
  pose.orientation.w = 1.0;

  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.4; //x
  primitive.dimensions[1] = 0.8; //y
  primitive.dimensions[2] = 0.001;  //z

  co.primitives.push_back(primitive);
  co.primitive_poses.push_back(pose);
  
  std::vector<moveit_msgs::CollisionObject> blocks;
  blocks.push_back(co);
  //ROS_INFO("Adding pole object to the wrold!");
  planning_scene_interface.addCollisionObjects(blocks);
  //-----------------------------------------------------
  // Planning to a Pose goal  
  geometry_msgs::Pose start_pose;
    start_pose.position.x    = 0.20625;
    start_pose.position.y    = -0.14628;
    start_pose.position.z    = 0.022625;
    start_pose.orientation.x = 0.0021168;
    start_pose.orientation.y = 0.0315464;
    start_pose.orientation.z = -0.306499;  
    start_pose.orientation.w = 0.951346;
  
   // Adding Path constraints
  moveit_msgs::PositionConstraint p_c;
  p_c.link_name = "gripper_guide_link";
  p_c.header.frame_id = "base_footprint";
  p_c.target_point_offset.z = 0.02;  
  p_c.weight = 1.0;

  moveit_msgs::Constraints test_constraints;
  test_constraints.position_constraints.push_back(p_c);
  move_group.setPathConstraints(test_constraints);
   // -------end Path Constraints----


  move_group.setPoseTarget(start_pose);  
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  // END_TUTORIAL

  ros::shutdown();
  return 0;
}
