
#include <ros/ros.h>
#include "phantomx_myanmar_custom_msg/custom.h"

int main( int argc, char **argv )
{

  ros::init( argc, argv, "myanmar_pick_pose_publish" );
  ros::NodeHandle n;
  
  //ros::Publisher pub_pick_pose = n.advertise< geometry_msgs::Pose >( "myanmar_pick_pose", 1000 );
  ros::ServiceClient client = n.serviceClient<phantomx_myanmar_custom_msg::custom>("myanmar_pick_pose");

  phantomx_myanmar_custom_msg::custom srv;

  //srv.request.A.position.x = 0.262725;
  //srv.request.A.position.y = 0.355331;
  //srv.request.A.position.z = 0.337447;
  //srv.request.A.orientation.x = -0.0919282;
  //srv.request.A.orientation.y = 0.704351;
  //srv.request.A.orientation.z = 0.0919449;
  //srv.request.A.orientation.w = 0.697843;

  srv.request.A.position.x = 0.23151;
  srv.request.A.position.y = 0.0291151;
  srv.request.A.position.z = 0.0553416;
  srv.request.A.orientation.x = -0.0666109;
  srv.request.A.orientation.y = 0.696314;
  srv.request.A.orientation.z = 0.0669603;
  srv.request.A.orientation.w = 0.711496;
  
  if( client.call(srv) )
  {
    ROS_INFO("Requesting ...");
    ROS_INFO("Status : %s ", (srv.response.status?"OK":"failed") );
  }
  else
  {
    ROS_ERROR("Fail to call service!");
    return -1;
  }
    

  //ros::shutdown();
  return 0;

}

