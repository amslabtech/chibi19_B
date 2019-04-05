#include "ros/ros.h"
#include "roomba_500driver_meiji/RoombaCtrl.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "math.h"
#include <tf/tf.h>
#include <iostream>
#include <fstream>

using namespace std;

float rundist = 0.0;
float theta = 0.0;

void odometrycallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    nav_msgs::Odometry _msg = *msg;
    rundist = sqrt( pow(_msg.pose.pose.position.x, 2.0) + pow(_msg.pose.pose.position.y, 2.0));
    theta = tf::getYaw(_msg.pose.pose.orientation);
}

int main(int argc, char **argv)
{
    int status = 0;
    int count = 0;
    float buf_rundist = 0.0;
    float speed_sum = 0.0;
    float speed = 0.0;

    ofstream outputfile("/home/amsl/Desktop/speed.txt");
    ros::init(argc, argv, "roatation");
    ros::NodeHandle roomba_ctrl_pub;
    ros::NodeHandle roomba_odometry_sub;
    ros::Publisher ctrl_pub = roomba_ctrl_pub.advertise<roomba_500driver_meiji::RoombaCtrl>("roomba/control", 1);
    ros::Subscriber odometry_sub = roomba_odometry_sub.subscribe("/roomba/odometry", 1, odometrycallback);
    ros::Rate loop_rate(10);
    roomba_500driver_meiji::RoombaCtrl msg;
    msg.mode = 11;
    msg.cntl.linear.x = 0.00;
    msg.cntl.angular.z = 0.00;

    while (ros::ok())
    {

        switch(status){
            case 0:
                if(count%20 == 0){
                    speed = speed_sum / 10.0;
                    outputfile<<speed<<"\n";
                    msg.cntl.linear.x = (float)count / 200.0;
                }
                if(count%20 > 10){
                    speed_sum += (rundist - buf_rundist) / 0.10;
                }
                buf_rundist = rundist;
                if(msg.cntl.linear.x > 1.00){
                    count=0;
                    ROS_INFO("dist : %f", rundist);
                    status++;
                }
                break;

            case 1:
		msg.cntl.linear.x = 0.00;
                msg.cntl.angular.z = 0.00;
                 break;
            default:
                ROS_INFO("Error. status : %d", status);
        }
        count++;
        ctrl_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

