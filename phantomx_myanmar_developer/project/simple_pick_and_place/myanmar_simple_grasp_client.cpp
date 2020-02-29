#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <phantomx_myanmar_custom_msg/PickAndPlaceAction.h>
#include <geometry_msgs/PoseArray.h>

const std::string arm_link = "base_footprint";
const double gripper_open = 0.03;
const double gripper_closed = 0.014;

const double z_up = 0.08;
const double z_down = -0.05;

const double block_size = 0.02;


class PhantomxMyanmar
{
private:
    
  ros::NodeHandle nh_;
  
  // Actions
  actionlib::SimpleActionClient<phantomx_myanmar_custom_msg::PickAndPlaceAction> pick_and_place_action_;
  phantomx_myanmar_custom_msg::PickAndPlaceGoal pick_and_place_goal_;
  
  geometry_msgs::Pose old_pose_;
  ros::Subscriber sub_;

public:

  PhantomxMyanmar() :
    pick_and_place_action_("pick_and_place", true)
  {
    // Initialize goals
    pick_and_place_goal_.frame = arm_link;
    pick_and_place_goal_.z_up = z_up;
    pick_and_place_goal_.gripper_open = gripper_open;
    pick_and_place_goal_.gripper_closed = gripper_closed;
    
    ROS_INFO("Finished initializing, waiting for servers...");
    
    pick_and_place_action_.waitForServer();
    
    ROS_INFO("Found servers.");

    
    //sub_ = nh_.subscribe("/pick_place_position",10, &PhantomxMyanmar::moveBlock, this);
    moveblock();   
   
  }
  
  /*
  void moveBlock(const geometry_msgs::PoseArrayConstPtr& pose_array)
  {
    geometry_msgs::Pose start_pose,end_pose;
    start_pose = pose_array->poses[0];
    end_pose   = pose_array->poses[1];
    pick_and_place_goal_.pickup_pose = start_pose;
    pick_and_place_goal_.place_pose = end_pose;
    pick_and_place_action_.sendGoalAndWait(pick_and_place_goal_, ros::Duration(30.0), ros::Duration(30.0));
    //pick_and_place_action_.sendGoal(pick_and_place_goal_);
    //pick_and_place_action_.waitForResult(ros::Duration(30.0)); 
    
  } 
  */
  void moveblock()
  {
    // no orientation xyz there is no plan! 
    // these are not plannable pose!
    /* not working
    geometry_msgs::Pose start_pose; 
    start_pose.position.x = 0.22;
    start_pose.position.y = -0.08;
    start_pose.position.z = 0;
    start_pose.orientation.w = 1.0;

    geometry_msgs::Pose end_pose;
    end_pose.position.x = 0.22;
    end_pose.position.y = 0.08;
    end_pose.position.z = 0;
    end_pose.orientation.w = 1.0;
  */
    
    geometry_msgs::Pose start_pose;
    start_pose.position.x    = 0.251924;
    start_pose.position.y    = -0.121327;
    start_pose.position.z    = 0.0224861;
    start_pose.orientation.x = -0.0155204;
    start_pose.orientation.y = 0.000969792;
    start_pose.orientation.z = -0.225593;  
    start_pose.orientation.w = 0.974098;

    geometry_msgs::Pose end_pose;
    end_pose.position.x    = 0.238876;
    end_pose.position.y    = 0.225669;
    end_pose.position.z    = 0.145345;
    end_pose.orientation.x = 0.016885;
    end_pose.orientation.y = -0.0427924;
    end_pose.orientation.z = 0.366652;  
    end_pose.orientation.w = 0.92922;
    
    pick_and_place_goal_.pickup_pose = start_pose;
    pick_and_place_goal_.place_pose = end_pose;
    //pick_and_place_action_.sendGoalAndWait(pick_and_place_goal_, ros::Duration(30.0), ros::Duration(30.0));
    pick_and_place_action_.sendGoal(pick_and_place_goal_);
    pick_and_place_action_.waitForResult(ros::Duration(30.0)); 
    
  } 
};

int main(int argc, char** argv)
{
  // initialize node
  ros::init(argc, argv, "myanmar_grasp_client");

  PhantomxMyanmar manip;

  // everything is done in cloud callback, just spin
  ros::spin();
}

