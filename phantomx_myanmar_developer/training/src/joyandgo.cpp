#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>


float shoulder_yaw_joint;
float shoulder_pitch_joint;
float elbow_pitch_joint;
float wrist_pitch_joint;
float gripper_revolute_joint;



float step = 0.01;

float axes1 = 0.0;
float axes2 = 0.0;
float axes3 = 0.0;
float axes4 = 0.0;
float axes5 = 0.0;

void jscallback(const sensor_msgs::JointState &js)
{
	shoulder_yaw_joint = js.position[0];
	shoulder_pitch_joint = js.position[1];
	elbow_pitch_joint = js.position[2];
	wrist_pitch_joint = js.position[3];
	gripper_revolute_joint = js.position[4];
}
void joycallback(const sensor_msgs::Joy &joy)
{
	axes1 = joy.axes[1];  axes2 = joy.axes[2];  axes3 = joy.axes[3];  axes4 = joy.axes[4];  axes5 = joy.axes[5];
}

int main(int argc,char** argv)
{
	ros::init(argc,argv,"joyandgo");
	ros::NodeHandle nh;
	ros::Publisher s_yaw = nh.advertise<std_msgs::Float64>("/shoulder_yaw_joint/command",50); // axes2
	ros::Publisher s_pic = nh.advertise<std_msgs::Float64>("/shoulder_pitch_joint/command",50);   // axes3
	ros::Publisher e_pic = nh.advertise<std_msgs::Float64>("/elbow_pitch_joint/command",2);   // axes1
	ros::Publisher w_pic = nh.advertise<std_msgs::Float64>("/wrist_pitch_joint/command",50);   // axes4
	ros::Publisher grip  = nh.advertise<std_msgs::Float64>("/gripper_revolute_joint/command",50);   // axes5

	ros::Subscriber joy_sub = nh.subscribe("/joy",50,joycallback);
	ros::Subscriber joint_sub = nh.subscribe("/joint_states",50,jscallback);

	ros::Rate r(5);

	while(ros::ok())
	{

		if(axes1 != 0)
		{
			std_msgs::Float64 epic;
			if(axes1 == 1) 
			{
				epic.data = elbow_pitch_joint+step;
				e_pic.publish(epic);
			}
			else if (axes1 == -1) 
			{ 
				epic.data = elbow_pitch_joint-step;
				e_pic.publish(epic);
			}
		}
		//----------------------------------------------------------
		if(axes2 != 0)
		{
			std_msgs::Float64 syaw;
			if(axes2 > 0) 
			{
				syaw.data = shoulder_yaw_joint+step;
				s_yaw.publish(syaw);
			}
			else 
			{ 
				syaw.data = shoulder_yaw_joint-step;
				s_yaw.publish(syaw);
			}
		}
		//-----------------------------------------------------------

		if(axes3 != 0)
		{
			std_msgs::Float64 spic;
			if(axes3 > 0) 
			{
				spic.data = shoulder_pitch_joint+step;
				s_pic.publish(spic);
			}
			else 
			{ 
				spic.data = shoulder_pitch_joint-step;
				s_pic.publish(spic);
			}
		}
		//------------------------------------------------------------
		if(axes4 != 0)
		{
			std_msgs::Float64 wpic;
			if(axes4 > 0) 
			{
				wpic.data = wrist_pitch_joint+step;
				w_pic.publish(wpic);
			}
			else 
			{ 
				wpic.data = wrist_pitch_joint-step;
				w_pic.publish(wpic);
			}
		}
		//------------------------------------------------------------
		if(axes5 != 0)
		{
			std_msgs::Float64 gg;
			if(axes5 > 0) 
			{
				gg.data = gripper_revolute_joint+step;
				grip.publish(gg);
			}
			else 
			{ 
				gg.data = gripper_revolute_joint-step;
				grip.publish(gg);
			}
		}
		r.sleep();
		ros::spinOnce();
	}


	return 0;


}