#include "ros/ros.h"
#include "roomba_500driver_meiji/RoombaCtrl.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "roomba_ctrl");
  ros::NodeHandle n;
  ros::Publisher ctrl_pub = n.advertise<roomba_500driver_meiji::RoombaCtrl>("roomba/control", 1000);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    roomba_500driver_meiji::RoombaCtrl msg;

    msg.mode = 11;
    msg.cntl.angular.z = 0.50;

    ctrl_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}

