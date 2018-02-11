/* Author : GhostMan */
#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group_interface.h>

// Robot state pubishing
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayRobotState.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "state_display_node");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	/* Load the robot model */
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");

	moveit::planning_interface::MoveGroupInterface move_group("arm");

	// Raw pointers are frequently used to refer to the planning group for improved performance.
  	const robot_state::JointModelGroup *joint_model_group =
    	move_group.getCurrentState()->getJointModelGroup("arm");
	
	// Getting Basic Information
	ROS_INFO_STREAM("Reference frame : "<< move_group.getPlanningFrame().c_str() );
	ROS_INFO_STREAM("End Effector : "<< move_group.getEndEffectorLink().c_str() );

	const Eigen::Affine3d &end_effector_state = move_group.getCurrentState()->getGlobalLinkTransform("gripper_guide_link");

  	//ROS_INFO_STREAM("-------------------------------------------------------");
  	//ROS_INFO_STREAM("Translation  X :" << end_effector_state.translation().x());
  	//ROS_INFO_STREAM("             Y :" << end_effector_state.translation().y());
  	//ROS_INFO_STREAM("             Z :" << end_effector_state.translation().z());
  	//ROS_INFO_STREAM("Rotation       :" << end_effector_state.rotation() );
  	//ROS_INFO_STREAM("-------------------------------------------------------");

  	ROS_INFO_STREAM("-------------------------------------------------------");
  	ROS_INFO_STREAM("Current Pose : "<< move_group.getCurrentPose() );
  	ROS_INFO_STREAM("-------------------------------------------------------");

  	bool found_ik = move_group.getCurrentState()->setFromIK(joint_model_group, end_effector_state, 10, 0.1);

  	// Now, we can print out the IK solution (if found):
  	if (found_ik)
  	{
  		ROS_INFO("IK solution Found");
  	}
  	else 
  	{
  		ROS_INFO("Did not find IK solution");
  	}


  	ros::shutdown();
  	return 0;
}