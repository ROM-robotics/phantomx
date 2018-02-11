#include <ros/ros.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <geometric_shapes/solid_primitive_dims.h>

#include <geometry_msgs/PoseArray.h>

static const std::string ROBOT_DESCRIPTION = "robot_description";




int main(int argc, char** argv)
{
	ros::init(argc, argv, "add_collision_object");
	ros::AsyncSpinner spinner(1.0);
	spinner.start();
	ros::NodeHandle nh;

	ros::Publisher pub = nh.advertise<geometry_msgs::PoseArray>("/pick_place_position",10);
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	sleep(5.0);
	
	
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
  primitive.dimensions[2] = 0.0001;  //z

  co.primitives.push_back(primitive);
  co.primitive_poses.push_back(pose);
  
  std::vector<moveit_msgs::CollisionObject> blocks;
  blocks.push_back(co);
  //ROS_INFO("Adding pole object to the wrold!");
  planning_scene_interface.addCollisionObjects(blocks);
  //-----------------------------------------------------
	//sleep(1.0);
	/* publish to grasp client */
	geometry_msgs::PoseArray poses;
	poses.poses.resize(2);
	poses.poses[0] = co.primitive_poses[0];
	/* tricky */
	poses.poses[1].position.x = 0.22;
	poses.poses[1].position.y = 0.8;
	poses.poses[1].position.y = 0;
	poses.poses[1].orientation.w = 1.0;
	
	ros::Rate r(1);
	while(ros::ok()){ pub.publish(poses); r.sleep();}
	
	

	
	// End tuto
	ros::WallDuration(1.0).sleep();
	ROS_INFO("End! I will shutdown now.");
	ros::shutdown();
	return 0;
}

