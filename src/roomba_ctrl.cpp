#include "ros/ros.h"
#include "roomba_500driver_meiji/RoombaCtrl.h"

#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "roomba_ctrl");
  ros::NodeHandle n;
  ros::Publisher ctrl_pub = n.advertise<roomba_500driver_meiji::RoombaCtrl>("roomba/control", 1000);

  ros::Rate loop_rate(10);
  int count = 0;
  while (ros::ok())
  {
    roomba_500driver_meiji::RoombaCtrl msg;

    //std::stringstream ss;
    //ss << "hello world " << count;
	msg.mode = 11;
    msg.cntl.linear.x = 0.02;

    ROS_INFO("%d", count);

	ctrl_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
  //test

  return 0;
}


