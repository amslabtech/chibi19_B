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
    float buf_theta = 0.0;
    float omega = 0.0;
    float domega = 0.0;

    ofstream outputfile("/home/amsl/Desktop/domega.txt");
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
                msg.cntl.linear.x = 0.00;
                msg.cntl.angular.z = 1.00;
                if(buf_theta > 0 && theta < 0){
                    omega = ( 2*M_PI - buf_theta + theta) / 0.10;
                    domega = 2 * ( (2*M_PI - buf_theta + theta) / 0.01 - omega / 0.10);
                }else{
                    omega = (theta - buf_theta) / 0.10;
                    domega = 2 * ( (theta - buf_theta) / 0.01 - omega / 0.10);
                }
                outputfile<<domega<<"\n";
                buf_theta = theta;
                if(count > 100){
                    count=0;
                    ROS_INFO("theta : %f", theta);
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

