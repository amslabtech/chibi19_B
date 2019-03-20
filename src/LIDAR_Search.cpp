#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "roomba_500driver_meiji/RoombaCtrl.h"
#include "math.h"
#include "chibi19_b/roomba.h"

struct LaserData{
  float angle; //[rad]
  float range; //[m]
};

int check = 0;
int state = 0;

void chatterCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  sensor_msgs::LaserScan _msg = *msg;
  
  const int N = int((_msg.angle_max - _msg.angle_min) / _msg.angle_increment);
  float s = 0;
  float count = 0;
  LaserData Ldata[N];
  for(int i=0; i<N; i++){
    Ldata[i].angle = _msg.angle_min + i*_msg.angle_increment;
    Ldata[i].range = _msg.ranges[i];
  }

  for(int i=0; i<N; i++){
    if(abs(Ldata[i].angle) < 0.523599){
      s += Ldata[i].range;
      count += 1.0;
    }else if(Ldata[i].angle >0.523599){
      break;
    }else{
      continue;
    }
  }

  if(s/count - 0.70 <= 0.05)
  {
	  check = 1;
  }

  if(s/count - 0.50 <= 0.05)
   {
       check = 2;
   }
// ROS_INFO("average: [%f]", s/count);
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

void state_publish()
{
	if(check ==1)
	{
		state = 5;
	}

	if(check = 2)
	{
		state = 6;
	}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "LIDAR");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("scan", 1000, chatterCallback);

  ros::Publisher state_pub = n.advertise<chibi19_b::roomba>("status", 1000);

  ros::Rate loop_rate(10);

  chibi19_b::roomba msg;

  while(ros::ok())
  {
	  state_publish();
	  msg.status = state;
	  state_pub.publish(msg);
	  ros::spinOnce();

	  loop_rate.sleep();
  }
  
  ros::spin();

  return 0;
}
