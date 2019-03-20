#include "ros/ros.h"
#include "roomba_500driver_meiji/RoombaCtrl.h"
#include "chibi19_b/roomba.h"

int status = 0;

void state_callback(const chibi19_b::roomba::ConstPtr& msg)
{
	chibi19_b::roomba _msg = *msg;
	status = _msg.status;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle _state_sub;
	ros::NodeHandle roomba_ctrl_pub;
	ros::Subscriber state_sub = _state_sub.subscribe("status", 1, state_callback);
    ros::Publisher ctrl_pub = roomba_ctrl_pub.advertise<roomba_500driver_meiji::RoombaCtrl>("roomba/control", 1);
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        roomba_500driver_meiji::RoombaCtrl msg;

        msg.mode = 11;
        switch(status){
            case 0:
                msg.cntl.linear.x = 0.20;
                msg.cntl.angular.z = 0.00;
                break;

			case 1:
				msg.cntl.linear.x = 0.1;
                 break;

            case 2:
                msg.cntl.linear.x = 0.00;
                msg.cntl.angular.z = 0.50;
                break;
            case 3:
                msg.cntl.linear.x = 0.00;
                msg.cntl.angular.z = 0.20;
                break;
            case 4:
                msg.cntl.linear.x = 0.20;
                msg.cntl.angular.z = 0.00;
                break;

			case 5:
                msg.cntl.linear.x = 0.05;
                msg.cntl.angular.z = 0.00;
                break;

            case 6:
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

