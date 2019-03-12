//
//  Localization.cpp
//  
//
//  Created by 吉内航 on 2019/03/08.
//

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "nav_msgs/OccupancyGrid.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"

#include <math.h>
#include <random>

//乱数の生成
std::random_device rnd;
std::mt19937 mt(rnd());
std::uniform_real_distribution<> rand1(0.0, 1.0);    //0<p<1の範囲で乱数を生成

nav_msgs::OccupancyGrid map;
geometry_msgs::PoseArray poses;


const int N = 5;

class Particle
{
public:
    Particle(void);
    void p_init(nav_msgs::OccupancyGrid&);
    void motion_update();
    void measurement_update();
    
    double weight;
    
    geometry_msgs::PoseStamped pose;
    
private:
};
std::vector<Particle> Particles;
void map_callback(const nav_msgs::OccupancyGridConstPtr& msg)
{
    map = *msg;
    
    for(int i = 0;i < N;i++)
    {
        Particle p;
		p.p_init(map);
        Particles.push_back(p);
        poses.poses.push_back(p.pose.pose);
    	ROS_INFO("%f\n",p.pose.pose.position.x);
		ROS_INFO("%f\n",p.pose.pose.position.y);
		ROS_INFO("%f\n",tf::getYaw(p.pose.pose.orientation));
	}
    poses.header.frame_id = 'map';


}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"localization");
    ros::NodeHandle nh;
    ros::NodeHandle local_nh("~");
    
    ros::Subscriber map_sub = nh.subscribe("/map",100,map_callback);

	ros::Rate rate(10.0);

	while(ros::ok())
	{
		int i = 0;
		map_callback(&map);
	}

	return 0;
}


Particle::Particle(void)
{
    pose.pose.position.x = 0.0;
    pose.pose.position.y = 0.0;
    quaternionTFToMsg(tf::createQuaternionFromYaw(0), pose.pose.orientation);
    weight = 1/(double)N;
}

void Particle::p_init(nav_msgs::OccupancyGrid& map_data)
{
    pose.pose.position.x = rand1(mt)*map_data.info.width;
    pose.pose.position.y = rand1(mt)*map_data.info.height;
    quaternionTFToMsg(tf::createQuaternionFromYaw(2*M_PI*rand1(mt)), pose.pose.orientation);
}
