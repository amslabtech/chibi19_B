#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"


void chattercallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	nav_msgs::Odometry  _msg = *msg;
	ROS_INFO("pose\n");
	ROS_INFO("x:%f\n", _msg.pose.pose.position.x);
	ROS_INFO("y:%f\n", _msg.pose.pose.position.y);
	ROS_INFO("z:%f\n\n", _msg.pose.pose.position.z);
	ROS_INFO("orientation\n");
	ROS_INFO("x:%f\n",_msg.pose.pose.orientation.x);
	ROS_INFO("y:%f\n",_msg.pose.pose.orientation.y);
	ROS_INFO("z:%f\n",_msg.pose.pose.orientation.z);
	ROS_INFO("w:%f\n",_msg.pose.pose.orientation.w);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener_roomba_Odmetry");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("chatter", 1000, chattercallback);

  ros::spin();

  return 0;
}

