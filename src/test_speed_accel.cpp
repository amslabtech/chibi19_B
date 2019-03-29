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
    ofstream change_x("./speed.txt");
    ofstream change_z("./roatate.txt");
    ros::init(argc, argv, "roatation");
    ros::NodeHandle roomba_ctrl_pub;
    ros::NodeHandle roomba_odometry_sub;
    ros::Publisher ctrl_pub = roomba_ctrl_pub.advertise<roomba_500driver_meiji::RoombaCtrl>("roomba/control", 1);
    ros::Subscriber odometry_sub = roomba_odometry_sub.subscribe("/roomba/odometry", 1, odometrycallback);
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        roomba_500driver_meiji::RoombaCtrl msg;

        msg.mode = 11;
        switch(status){
            case 0:
                msg.cntl.linear.x = 0.00;
                msg.cntl.angular.z = 0.00;
                if(count%100 == 0){
                    msg.cntl.linear.x = (float)count / 1000;
                }
                change_x<<"x:\t"<<msg.cntl.linear.x<<"\t";
                change_x<<"rundist:\t"<<rundist<<"\n";
                if(msg.cntl.linear.x > 1.00){
                    count=0;
                    status++;
                }
                break;

            case 1:
		msg.cntl.linear.x = 0.00;
                msg.cntl.angular.z = 0.00;
		if(count > 300 ){
                     status++;
                 }
                 break;

            case 2:
                msg.cntl.linear.x = 0.00;
                msg.cntl.angular.z = 0.00;
                if(count%100 == 0){
                    msg.cntl.linear.z = (float)count / 1000;
                }
                change_z<<"z:\t"<<msg.cntl.angular.z<<"\t";
                change_z<<"theata:\t"<<theta<<"\n";
                if(msg.cntl.angular.z > 1.00){
                    count=0;
                    status++;
                }
                break;
            case 3:
                msg.cntl.linear.x = 0.00;
                msg.cntl.angular.z = 0.00;
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

