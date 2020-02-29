#!/usr/bin/env python
import rospy 
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseArray, PoseStamped, Pose, Quaternion
from moveit_commander import PlanningSceneInterface
import moveit_msgs.msg
import sys
import copy
import numpy


rospy.init_node("Pick_Place_Pose_Python")
pub   = rospy.Publisher("pick_place_position", PoseArray, queue_size=10)
scene = PlanningSceneInterface()


def add_block(name):
	p = PoseStamped()
	p.header.frame_id = "base_footprint"
	p.header.stamp    = rospy.Time.now()
	p.pose.position.x = 0.22
	p.pose.position.y = -0.08
	p.pose.position.z = 0.1

	q = quaternion_from_euler(0.0,0.0,0.0)
	p.pose.orientation = Quaternion(*q)
	add_box( name, p, (0.02,0.02,0.02) )

	return p.pose

def add_table(name):
	p = PoseStamped()
	p.header.frame_id = "base_footprint"
	p.header.stamp    = rospy.Time.now()
	p.pose.position.x = 0.22
	p.pose.position.y = -0.08
	p.pose.position.z = 0.1

	q = quaternion_from_euler(0.0,0.0,0.0)
	p.pose.orientation = Quaternion(*q)
	add_box( name, p, (0.1,0.1,0.1) )

	return p.pose


def add_box(name, p, size):
	collision_object = moveit_msgs.msg.CollisionObject()
	collision_object.header.frame_id = "base_footprint"
	collision_object.id = name

	primitive = SolidPrimitive()
	primitive.type = primitive.BOX
	primitive.dimensions.resize(3)
	primitive.dimensions[0] = size(0)
	primitive.dimensions[1] = size(1)
	primitive.dimensions[2] = size(3)

	collision_object.primitives.push_back(primitive)
	collision_object.primitive_poses.push_back(p)
	collision_object.operation = collision_object.ADD

	scene.add_collision_objects(collision_objects);
#Pose1 = add_block("block")
Pose2 = add_table("table")


rate = ros.Rate(1)
while not rospy.is_shutdown():
	pose_array = PoseArray()
	pose_array.poses.append(Pose2)

	#Pose2 = Pose()	
	#Pose2.position.x = 0.3
	#Pose2.position.y = 0
	#Pose2.position.z = 0.1
	#Pose2.orientation.w = 1.0

	#msg.poses.append(Pose2)


	pub.publish(pose_array)
	rate.sleep()