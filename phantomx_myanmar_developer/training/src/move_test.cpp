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

  
  static const std::string PLANNING_GROUP = "arm";

  
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_footprint");
  visual_tools.deleteAllMarkers();

 
  visual_tools.loadRemoteControl();

  
  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  
  visual_tools.trigger();

  
  ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());

  
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());

  
  
 
  geometry_msgs::Pose home; 
  //target_pose1.orientation.w = 0.0; 
  //target_pose1.position.x    = 0.28;
  //target_pose1.position.y    = -0.2; 
  //target_pose1.position.z    = 0.5;
  move_group.setPoseTarget(home);


  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  

  
  move_group.move();
  ROS_INFO("move");

  ROS_INFO("move group interface tuto will SHUTDOWN!");
  ros::waitForShutdown();
  return 0;
}