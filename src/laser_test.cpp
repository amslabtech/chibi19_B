#include "ros/ros.h"
#include "roomba_500driver_meiji/RoombaCtrl.h"
#include "sensor_msgs/LaserScan.h"

struct LaserData{
    float angle;
    float range;
};

void lasercallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    sensor_msgs::LaserScan _msg = *msg;

    const int N = int((_msg.angle_max - _msg.angle_min) / _msg.angle_increment);
    
	ROS_INFO("max = %f, min = %f, increment = %f", _msg.angle_max, _msg.angle_min, _msg.angle_increment);
	
	float s = 0.0;
    float count = 0.0;
    LaserData Ldata[N];
    for(int i=0; i<N; i++){
        Ldata[i].angle = _msg.angle_min + i*_msg.angle_increment;
        Ldata[i].range = _msg.ranges[i];

		if(Ldata[i].range < 0.25){
			ROS_INFO("Ldata[%d].range = %.3f", i, Ldata[i].range);
		}
    }

}

int main(int argc, char **argv){

	ros::init(argc, argv, "laser_test");

	ros::NodeHandle roomba_ctrl_pub;
	ros::NodeHandle scan_laser_sub;
	ros::Publisher ctrl_pub = roomba_ctrl_pub.advertise<roomba_500driver_meiji::RoombaCtrl>("roomba/control", 1);
	ros::Subscriber laser_sub = scan_laser_sub.subscribe("scan", 1, lasercallback);
	ros::Rate loop_rate(10);

	while(ros::ok){
		roomba_500driver_meiji::RoombaCtrl msg;
		ctrl_pub.publish(msg);
        ros::spinOnce();
		loop_rate.sleep();
	}
}
