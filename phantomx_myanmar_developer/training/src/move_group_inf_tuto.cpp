#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char **argv)
{
	ros::init(argc,argv,"move_group_interface_tuto");
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	// Begin tutorial
	static const std::string PLANNING_GROUP = "arm";
	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

	// Visualization
	namespace rvt = rviz_visual_tools;
	moveit_visual_tools::MoveItVisualTools visual_tools("base_footprint");
	visual_tools.deleteAllMarkers();
	// remote control is introspection tools that allows user to to step
	visual_tools.loadRemoteControl();

	Eigen::Affine3d test_pose = Eigen::Affine3d::Identity();
	test_pose.translation().z() = 0.75; // above head of robot
	visual_tools.publishText(test_pose,"MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

	visual_tools.trigger();

	// Getting Basic Information
	ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());
	ROS_INFO_NAMED("tutorial", "End Effector link: %s",move_group.getEndEffectorLink().c_str());

	// Planning to a Pose goal
	geometry_msgs::Pose target_pose1;
	target_pose1.orientation.x = 0.30459;
	target_pose1.orientation.y = 0.29272;
	target_pose1.orientation.z = -0.51857;	
	target_pose1.orientation.w = 0.74339;	
	target_pose1.position.x    = 0.092261;
	target_pose1.position.y    = -0.18406; 
	target_pose1.position.z    = 0.17335;

  
	move_group.setPoseTarget(target_pose1);

	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

	ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success?"":"FAILED");

	// Visualizing plans 
	ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
	visual_tools.publishAxisLabeled(target_pose1, "pose1");
	visual_tools.publishText(test_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
	visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
	visual_tools.trigger();
	//visual_tools.prompt("next step");

	/* Uncomment below line when working with a real robot */
	move_group.move();
	ROS_INFO("move");
	



	ROS_INFO("move group interface tuto will SHUTDOWN!");
	ros::waitForShutdown();
	return 0;
}