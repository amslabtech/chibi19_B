#include "ros/ros.h"
#include "roomba_500driver_meiji/RoombaCtrl.h"
#include "nav_msgs/Odometry.h"
#include "math.h"

float distance = 0;
float theata = 0;

void chattercallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    nav_msgs::Odometry _msg = *msg;
    distance = sqrt( pow(_msg.pose.pose.position.x, 2.0) + pow(_msg.pose.pose.position.y, 2.0));
    theata = asin(_msg.pose.pose.orientation.z);
}



int main(int argc, char **argv)
{
    int status = 0;
    ros::init(argc, argv, "roatation");
    ros::NodeHandle n;
    ros::Publisher ctrl_pub = n.advertise<roomba_500driver_meiji::RoombaCtrl>("roomba/control", 1);
    ros::Rate loop_rate(10);
    ros::NodeHandle roomba_odometry_sub;
    ros::Subscriber sub = roomba_odometry_sub.subscribe("/roomba/odometry", 1, chattercallback);
    while (ros::ok())
    {
        roomba_500driver_meiji::RoombaCtrl msg;

        msg.mode = 11;
        switch(status){
            case 0:
                msg.cntl.angular.z = 0.50;
                if(theata < -0.10){
                    status++;
                }
                break;
            case 1:
                msg.cntl.angular.z = 0.50;
                if(theata >= 0.0){
                    status++;
                }
                break;
            case 2:
                msg.cntl.angular.z = 0.0;
                break;
            default:
                ROS_INFO("Error. status : %d", status);
        }
        ctrl_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

