#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "chibi19_b/roomba.h"
#include "tf/tf.h"

int check = 0;
int state = 0;
double dist = 0;
double theta = 0.0;

void chattercallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	nav_msgs::Odometry  _msg = *msg;
	/*ROS_INFO("p.x:%f", _msg.pose.pose.position.x);
	ROS_INFO("p.y:%f", _msg.pose.pose.position.y);
	ROS_INFO("p.z:%f", _msg.pose.pose.position.z);
	ROS_INFO("o.x:%f",_msg.pose.pose.orientation.x);
	ROS_INFO("o.y:%f",_msg.pose.pose.orientation.y);
	ROS_INFO("o.z:%f",_msg.pose.pose.orientation.z);
	ROS_INFO("o.w:%f",_msg.pose.pose.orientation.w);*/
	
	dist = sqrt( pow(_msg.pose.pose.position.x, 2.0) + pow(_msg.pose.pose.position.y, 2.0));
	theta = tf::getYaw(_msg.pose.pose.orientation);

	if(theta <= -0.80)
	{
		check = 1;
	}

	if(abs(theta) <=0.01&&check == 1)
	{
		check = 2;
	}

	if(abs(dist - 2.8) <= 0.05)
	{
		check = 3;
	}

	if(abs(dist - 3.0) <= 0.05)
	{
		check = 4;
	}


}

void state_publish()
 {
     if(check == 0)
     {   
         state = 0;
     }
     
     if(check == 1)
     {   
         state = 3;
	 }

	 if(check == 2)
	 {
		 state = 4;
	 }

	 if(check == 3)
	 {
		 state = 1;
	 }

	 if(check == 4)
	 {
		 state = 2;
	 }
 }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener_roomba_Odometry");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/roomba/odometry", 1000, chattercallback);

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

