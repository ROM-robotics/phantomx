#include <ros/ros.h>
#include <tf/tf.h>

#include <actionlib/server/simple_action_server.h>
#include <phantomx_myanmar_custom_msg/PickAndPlaceAction.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/PoseArray.h>

// for gripper command in RViz
#include <sensor_msgs/JointState.h>
// for gripper command in Real Robot
#include <std_msgs/Float64.h>


namespace phantomx_myanmar_custom_msg
{

class PickAndPlaceServer
{
private:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<phantomx_myanmar_custom_msg::PickAndPlaceAction> as_;
  std::string action_name_;

  phantomx_myanmar_custom_msg::PickAndPlaceFeedback     feedback_;
  phantomx_myanmar_custom_msg::PickAndPlaceResult       result_;
  phantomx_myanmar_custom_msg::PickAndPlaceGoalConstPtr goal_;

  //ros::Publisher target_pose_pub_;
  ros::Subscriber pick_and_place_sub_;
  ros::Publisher gripper_commander_;

  // Move groups to control arm and gripper with MoveIt!
  moveit::planning_interface::MoveGroupInterface arm_;

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool status;

  // Pick and place parameters
  std::string arm_link;
  double gripper_open;
  double gripper_closed;
  double attach_time;
  double detach_time;
  double z_up;



public:
  PickAndPlaceServer(const std::string name) :
    nh_("~"), as_(name, false), action_name_(name), arm_("arm")
  {
    // Read specific pick and place parameters
    nh_.param("grasp_attach_time", attach_time, 1.5);
    nh_.param("grasp_detach_time", detach_time, 1.0);

    // Getting Basic Information
    // We can print the name of the reference frame for this robot.
    ROS_INFO("Reference frame: %s", arm_.getPlanningFrame().c_str());
    ROS_INFO("End effector link: %s", arm_.getEndEffectorLink().c_str());

    // Register the goal and feedback callbacks
    as_.registerGoalCallback(boost::bind(&PickAndPlaceServer::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&PickAndPlaceServer::preemptCB, this));

    as_.start();
    /* for real robot */
    gripper_commander_ = nh_.advertise<std_msgs::Float64>("/gripper_prismatic_joint/command", 1, true);
    
  }



  void goalCB()
  {    
    ROS_INFO("[pick and place] Received goal!");
    goal_ = as_.acceptNewGoal();
    arm_link = goal_->frame;
    gripper_open = goal_->gripper_open;
    gripper_closed = goal_->gripper_closed;
    z_up = goal_->z_up;

    arm_.setPoseReferenceFrame(arm_link);

    // Allow some leeway in position (meters) and orientation (radians)
    //arm_.setGoalPositionTolerance(0.001);
    //arm_.setGoalOrientationTolerance(0.1);

    // Allow replanning to increase the odds of a solution
    arm_.allowReplanning(true);

    pickAndPlace(goal_->pickup_pose, goal_->place_pose);
    
  }

  
  void preemptCB()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
  }

  void pickAndPlace(const geometry_msgs::Pose& start_pose, const geometry_msgs::Pose& end_pose)
  {
    ROS_INFO("pickAndPlace Function start!");

    //geometry_msgs::Pose target;
    setGripper(gripper_open);
    ros::Duration(detach_time).sleep();
    
    
    ROS_INFO("Setting Pose target pick_pose");    
    
    geometry_msgs::Pose up_pose = start_pose;    
    arm_.setPoseTarget(start_pose);
    
    status = (arm_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing plan start pose goal %s", status ? "OK" : "FAILED");
    
    if(status)
    {
      arm_.move();
      ros::Duration(attach_time).sleep();
      setGripper(gripper_closed);
      ros::Duration(attach_time).sleep();
      arm_.setPoseTarget(end_pose);
      // Plan again
      status = (arm_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      ROS_INFO("Visualizing plan place pose goal %s", status ? "" : "FAILED");
      
      if(status)
      {
        arm_.move();
        // detach the object
        ros::Duration(detach_time).sleep();
        setGripper(gripper_open);
        ros::Duration(detach_time).sleep();
      }
      else
      {
        return;
      } 
    }
    else
    {
      return;
    } 
    
    as_.setSucceeded(result_);
  }
  bool setGripper(float opening)
  {
    ROS_INFO("Sending Gripper command..");
    // publish gripper command for real robot 
    std_msgs::Float64 centiMeter;
    centiMeter.data = opening;
    gripper_commander_.publish(centiMeter);   
    return true;
  }/*
  void goHome()
  {
    geometry_msgs::Pose home;
    home.position.x    = 0.127039;
    home.position.y    = -0.00670322;
    home.position.z    = 0.15025;
    home.orientation.x = -0.00485531;
    home.orientation.y = 0.0078354;
    home.orientation.z = -0.0331896; 
    home.orientation.w = 0.999407;

    arm_.setPoseTarget(home);
    status = (arm_.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing plan goHome %s", status ? "OK" : "FAILED");
    if(status)
    {
      arm_.move();
    }
  } */
}; // class
}; // namespace



int main(int argc, char** argv)
{
  ros::init(argc, argv, "pick_and_place_action_server");

  phantomx_myanmar_custom_msg::PickAndPlaceServer server("pick_and_place");

  // Setup an multi-threaded spinner as the move groups operations need continuous spinning
  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();

  return 0;
}
