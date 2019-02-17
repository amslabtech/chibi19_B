#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"


void chattercallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	nav_msgs::Odometry  _msg = *msg;
	ROS_INFO("p.x:%f", _msg.pose.pose.position.x);
	ROS_INFO("p.y:%f", _msg.pose.pose.position.y);
	ROS_INFO("p.z:%f", _msg.pose.pose.position.z);
	ROS_INFO("o.x:%f",_msg.pose.pose.orientation.x);
	ROS_INFO("o.y:%f",_msg.pose.pose.orientation.y);
	ROS_INFO("o.z:%f",_msg.pose.pose.orientation.z);
	ROS_INFO("o.w:%f",_msg.pose.pose.orientation.w);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener_roomba_Odometry");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/roomba/odometry", 1000, chattercallback);

  ros::spin();

  return 0;
}

