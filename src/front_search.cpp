#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "roomba_500driver_meiji/RoombaCtrl.h"
#include "math.h"

typedef struct LaserData{
  float angle; //[rad]
  float range; //[m]
};

void chatterCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  sensor_msgs::LaserScan _msg = *msg;

  const int N = sizeof(_msg.ranges);
  int s = 0;
  int count = 0;
  LaserData Ldata[N];
  for(int i=0; i<N; i++){
    Ldata[i].angle = _msg.angle_min + i*_msg.angle_increment;
    Ldara[i].range = _msg.rabges[i];
  }

  for(int i=0; i<N; i++){
    if(abs(Ldata[i].angle) < 0.523599){
      s += Ldara[i].range;
      count++;
    }if else(Ldata[i].angle >0.523599){
      break;
    }else{
      continue;
    }
  }

  ROS_INFO("average: [%f]", s/count);




/*
  ROS_INFO("angle_min: [%f]", _msg.angle_min);
  ROS_INFO("angle_max: [%f]", _msg.angle_max);
  ROS_INFO("angle_increment: [%f]", _msg.angle_increment);
  ROS_INFO("time_increment: [%f]", _msg.time_increment);
  ROS_INFO("scan_time: [%f]", _msg.scan_time);
  ROS_INFO("range_min: [%f]", _msg.range_min);
  ROS_INFO("range_max: [%f]", _msg.range_max);
  for(int i=0; i<4; i++){
    ROS_INFO("ranges%d: [%f]", i, _msg.ranges[i]);
  }
*/
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("scan", 1000, chatterCallback);

  ros::spin();

  return 0;
}
