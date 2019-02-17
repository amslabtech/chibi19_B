#include "ros/ros.h"
#include "roomba_500driver_meiji/RoombaCtrl.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "math.h"

float rundist = 0.0;
float theata = 0.0;
float sensor_front_range = 0.0;

struct LaserData{
    float angle;
    float range;
};

void odometrycallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    nav_msgs::Odometry _msg = *msg;
    rundist = sqrt( pow(_msg.pose.pose.position.x, 2.0) + pow(_msg.pose.pose.position.y, 2.0));
    theata = asin(_msg.pose.pose.orientation.z);
}

void lasercallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    sensor_msgs::LaserScan _msg = *msg;

    const int N = int((_msg.angle_max - _msg.angle_min) / _msg.angle_increment);
    float s = 0.0;
    float count = 0.0;
    LaserData Ldata[N];
    for(int i=0; i<N; i++){
        if(abs(Ldata[i].angle) < (M_PI / 18)){
            s += Ldata[i].range;
            count += 1.0;
        }else if(Ldata[i].angle > (M_PI / 18)){
            break;
        }else{
            continue;
        }
    }

    sensor_front_range = s/count;

}



int main(int argc, char **argv)
{
    int status = 0;
    ros::init(argc, argv, "roatation");
    ros::NodeHandle roomba_ctrl_pub;
    ros::NodeHandle roomba_odometry_sub;
    ros::NodeHandle scan_laser_sub;
    ros::Publisher ctrl_pub = roomba_ctrl_pub.advertise<roomba_500driver_meiji::RoombaCtrl>("roomba/control", 1);
    ros::Subscriber odometry_sub = roomba_odometry_sub.subscribe("/roomba/odometry", 1, odometrycallback);
    ros::Subscriber laser_sub = scan_laser_sub.subscribe("scan", 1, lasercallback);
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        roomba_500driver_meiji::RoombaCtrl msg;

        msg.mode = 11;
        switch(status){
            case 0:
                msg.cntl.linear.x = 0.50;
                msg.cntl.angular.z = 0.00;
                if(rundist > 3.0){
                    status++;
                }
                break;
            case 1:
                msg.cntl.linear.x = 0.00;
                msg.cntl.angular.z = 0.50;
                if(theata < -0.10){
                    status++;
                }
                break;
            case 2:
                msg.cntl.linear.x = 0.00;
                msg.cntl.angular.z = 0.50;
                if(theata >= 0.0){
                    status++;
                }
                break;
            case 3:
                msg.cntl.linear.x = 0.50;
                msg.cntl.angular.z = 0.00;
                if(sensor_front_range <= 0.50){
                    status++;
                }
                break;
            case 4:
                msg.cntl.linear.x = 0.00;
                msg.cntl.angular.z = 0.00;
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

