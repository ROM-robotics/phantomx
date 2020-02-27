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
	
	//visual_tools.loadRemoteControl();

	Eigen::Affine3d test_pose = Eigen::Affine3d::Identity();
	test_pose.translation().z() = 0.75; // above head of robot

	//isual_tools.trigger();

	// Getting Basic Information
	ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());
	ROS_INFO_NAMED("tutorial", "End Effector link: %s",move_group.getEndEffectorLink().c_str());

	// Planning to a Pose goal
	geometry_msgs::Pose home;
	//target_pose1.orientation.x = 0.30459;
	//target_pose1.orientation.y = 0.29272;
	//target_pose1.orientation.z = 0.51857;	
	//target_pose1.orientation.w = 0.74339;	
	//target_pose1.position.x    = 0.0;
	//target_pose1.position.y    = -0.13; 
	//target_pose1.position.z    = 0.0;

  
	move_group.setPoseTarget(home);

	moveit::planning_interface::MoveGroupInterface::Plan plan;
	bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	if(success)
	{
		move_group.move();
		ROS_INFO("move");
	}
	else
	{
		ROS_INFO("NO MOVE");
	}

	
	//visual_tools.prompt("next step");

	/* Uncomment below line when working with a real robot */
	



	ROS_INFO("move group interface tuto will SHUTDOWN!");
	ros::waitForShutdown();
	return 0;
}