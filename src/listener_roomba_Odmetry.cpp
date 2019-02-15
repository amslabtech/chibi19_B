#include "ros/ros.h"
#include "std_msgs/String.h"
#include "roomba_500driver_meiji/RoombaCtrl.h"
#include "nav_msgs/Odmetry.h"

void chattercallback(const std_msgs::odmetry::constptr& msg)
{
	_msg = *msg
	ros_info("pose\n");
	ros_info("x:%f\n", pose.pose.position.x);
	ros_info("y:%f\n", pose.pose.position.y);
	ros_info("z:%f\n\n", pose.pose.position.z);
	ros_info("orientation\n");
	ros_info("x:%f\n"pose.pose.orientation.x);
	ros_info("y:%f\n"pose.pose.orientation.y);
	ros_info("z:%f\n"pose.pose.orientation.z);
	ros_info("w:%f\n"pose.pose.orientation.w);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener_roomba_Odmetry");

  ros::nodehandle n;

  ros::subscriber sub = n.subscribe("chatter", 1000, chattercallback);

  ros::spin();

  return 0;
}

