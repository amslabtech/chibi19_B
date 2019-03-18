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
sensor_msgs::LaserScan laser;

bool map_get = false;

class Particle
{
public:
    Particle(void);
    void p_init(double,double, double);
    void motion_update(geometry_msgs::PoseStamped, geometry_msgs::PoseStamped);
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

void laser_callback(sensor_msgs::LaserScanConstPtr& msg)
{
	laser = *msg;	
}

double Get_Yaw(const geometry_msgs::Quaternion q)
{
    double rool, pitch, yaw;
    tf::Quaternion quat(q.x,q.y,q.z,q.w);
    tf::Matrix3x3(quat).getRPY(rool, pitch, yaw);
    
    return yaw;
}

double cul_angle_diff(double a, double b)
{
    a = atan2(sin(a),cos(a));
    b = atan2(sin(b),cos(b));
    
    double d1 = a-b;
    double d2 = 2*M_PI-fabs(d1);
    
    if(fabs(d1) < fabs(d2))
    {
        return d1;
    }
    
    else if(d1>0)
    {
        return -1.0*d2;
    }
    
    else
        return d2;
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"localization");
    ros::NodeHandle nh;
    ros::NodeHandle local_nh("~");
    
    ROS_INFO("Started\n");
   
    ros::Subscriber map_sub = nh.subscribe("/map",100,map_callback);
    
    tf::TransformListener listener;
    tf::StampedTransform temp_tf_stamped;
    temp_tf_stamped = tf::StampedTransform(tf::Transform(tf::createQuaternionFromYaw(0.0), tf::Vector3(0.0, 0.0, 0)), ros::Time::now(), "map", "odom");

	ros::Rate rate(10.0);
	
	ros::spin();
    
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

void Particle::motion_update(geometry_msgs::PoseStamped current, geometry_msgs::PoseStamped previous)
{
    double dx,dy,dyaw;
    double delta;
    double dist;
    
    dx = current.pose.position.x - previous.pose.position.x;
    dy = current.pose.position.y - previous.pose.position.y;
    dyaw = cul_angle_diff(Get_Yaw(current.pose.orientation), Get_Yaw(previous.pose.orientation));
    
    dist = sqrt(dx*dx + dy*dy);
    
    if(dist < 0.01)
    {
        delta  = 0;
    }
    
    pose.pose.position.x += dist * cos(Get_Yaw(pose.pose.orientation)) + rand_nomal(0.0, x_cov);
    pose.pose.position.y += dist * sin(Get_Yaw(pose.pose.orientation)) + rand_nomal(0.0, y_cov);
    quaternionTFToMsg(tf::createQuaternionFromYaw(Get_Yaw(pose.pose.orientation) + dyaw + rand_nomal(0.0, yaw_cov)), pose.pose.orientation);
}
