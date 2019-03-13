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

bool map_get = false;

class Particle
{
public:
    Particle(void);
    void p_init(double,double, double);
    void motion_update();
    void measurement_update();
    
    double weight;
    
    geometry_msgs::PoseStamped pose;
    
private:
};

const int N = 5;
double init_x = 0.0;
double init_y = 0.0;
double init_yaw = 0.0;
double x_cov = 2.0;
double y_cov = 2.0;
double yaw_cov = 1.0;

std::vector<Particle> Particles;

void map_callback(const nav_msgs::OccupancyGridConstPtr& msg)
{
    ROS_INFO("getting map");
	map = *msg;
    
    for(int i = 0;i < N;i++)
    {
        Particle p;
		p.p_init(init_x, init_y, init_yaw);
        Particles.push_back(p);
        poses.poses.push_back(p.pose.pose);
		ROS_INFO("%f", p.pose.pose.position.x);
		ROS_INFO("%f", p.pose.pose.position.y);
		ROS_INFO("%f", tf::getYaw(p.pose.pose.orientation));
		ROS_INFO("\n");
	}
    poses.header.frame_id = "map";
	map_get = true;
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"localization");
    ros::NodeHandle nh;
    ros::NodeHandle local_nh("~");
   
    ros::Subscriber map_sub = nh.subscribe("/map",100,map_callback);

	ROS_INFO("Started\n");

	ros::Rate rate(10.0);
	
	while(ros::ok())
{
	ros::spin();

	if(map_get)
		ROS_INFO("Ready");
}
	return 0;
}

double rand_nomal(double mu, double sigma)
{
	double z = sqrt(-2.0*log(rand1(mt)))*sin(2.0*M_PI*rand1(mt));
	return mu + sigma*z;
}

Particle::Particle(void)
{
    pose.pose.position.x = 0.0;
    pose.pose.position.y = 0.0;
    quaternionTFToMsg(tf::createQuaternionFromYaw(0), pose.pose.orientation);
    weight = 1/(double)N;
}

void Particle::p_init(double x, double y, double theta)
{
    pose.pose.position.x = rand_nomal(x, x_cov);
    pose.pose.position.y = rand_nomal(y, y_cov);
    quaternionTFToMsg(tf::createQuaternionFromYaw(rand_nomal(theta, yaw_cov)), pose.pose.orientation);
}
