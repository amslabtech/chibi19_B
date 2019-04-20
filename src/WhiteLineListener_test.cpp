#include "ros/ros.h"
#include "std_msgs/Bool.h"


void chatterCallback(const std_msgs::Bool::ConstPtr& msg)
{
  ROS_INFO("I heard: [%d]", *msg);
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "WhiteLineListener_test");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("whiteline", 1000, chatterCallback);

  ros::spin();

  return 0;
}