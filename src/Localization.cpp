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
geometry_msgs::PoseStamped estimated_pose;
geometry_msgs::PoseStamped current_pose;
geometry_msgs::PoseStamped previous_pose;
geometry_msgs::PoseArray poses;
sensor_msgs::LaserScan laser;

bool map_get = false;

class Particle
{
public:
    Particle(void);
    void p_init(double,double, double);
    void motion_update(geometry_msgs::PoseStamped, geometry_msgs::PoseStamped, int);
    void measurement_update();
    
    double weight;
    
    geometry_msgs::PoseStamped pose;
    
private:
};

const int N = 10;
double init_x = 0.0;
double init_y = 0.0;
double init_yaw = 0.0;
double x_cov = 2.0;
double y_cov = 2.0;
double yaw_cov = 1.0;
double Max_Range = 20;
double w_slow = 0.0;
double w_fast = 0.0;
double sigma = 3.0;
double a_slow = 0.01;
double a_fast = 0.1;
double range_count = 3;


std::vector<Particle> Particles;

double Get_Yaw(const geometry_msgs::Quaternion);

void map_callback(const nav_msgs::OccupancyGridConstPtr& msg)
{
    ROS_INFO("getting map");
	map = *msg;
    
    for(int i = 0;i < N;i++)
    {
        Particle p;
		p.p_init(init_x, init_y, init_yaw);
		ROS_INFO("%f", p.pose.pose.position.x);
		ROS_INFO("%f", p.pose.pose.position.y);
		ROS_INFO("%f", tf::getYaw(p.pose.pose.orientation));
		ROS_INFO("\n");

		geometry_msgs::Pose particle_pose;

		particle_pose.position.x = p.pose.pose.position.x;
		particle_pose.position.y = p.pose.pose.position.y;
		particle_pose.position.z = 0.0;
		quaternionTFToMsg(tf::createQuaternionFromYaw(Get_Yaw(p.pose.pose.orientation)), particle_pose.orientation);

		poses.poses.push_back(particle_pose);
		Particles.push_back(p);
	}
    poses.header.frame_id = "map";	
	map_get = true;
}

void laser_callback(const sensor_msgs::LaserScanConstPtr& msg)
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

int get_index(double x, double y)
{
	int index, index_x, index_y;

	index_x = floor((x - map.info.origin.position.x) / map.info.resolution);
	index_y = floor((y - map.info.origin.position.y) / map.info.resolution)*map.info.width;

	index = index_x + index_y;

	return index;
}

bool update_judge(geometry_msgs::PoseStamped current, geometry_msgs::PoseStamped previous)
{
	if(sqrt(pow(current.pose.position.x-previous.pose.position.x,2)+pow(current.pose.position.y-previous.pose.position.y,2))<0.01)
	{
		return false;
	}

	else
	{
		return true;
	}
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"Localization");
    ros::NodeHandle nh;
    ros::NodeHandle local_nh("~");
    
    ROS_INFO("Started\n");
   
    ros::Subscriber map_sub = nh.subscribe("/map",100, map_callback);
	ros::Subscriber laser_sub = nh.subscribe("/scan",100, laser_callback);
	ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/chibi19/estimated_pose",100);
	ros::Publisher pose_array_pub = nh.advertise<geometry_msgs::PoseArray>("/poses",100);

	tf::TransformBroadcaster map_broadcaster;
	tf::TransformListener listener;
    tf::StampedTransform temp_tf_stamped;
    temp_tf_stamped = tf::StampedTransform(tf::Transform(tf::createQuaternionFromYaw(0.0), tf::Vector3(0.0, 0.0, 0)), ros::Time::now(), "map", "odom");

	current_pose.pose.position.x = 0.0;
	current_pose.pose.position.y = 0.0;
	quaternionTFToMsg(tf::createQuaternionFromYaw(0), current_pose.pose.orientation);

	ros::Rate rate(10.0);

	while(ros::ok())
	{
		if(map_get)// && !laser.ranges.empty())
		{
			estimated_pose.header.frame_id = "map";
			poses.header.frame_id = "map";

			tf::StampedTransform transform;
			transform = tf::StampedTransform(tf::Transform(tf::createQuaternionFromYaw(0.0), tf::Vector3(0.0, 0.0, 0)), ros::Time::now(), "odom", "base_link");

			try{
				listener.waitForTransform("odom", "base_link", ros::Time(0), ros::Duration(1.0));
				listener.lookupTransform("odom", "base_link", ros::Time(0), transform);
			}

			catch(tf::TransformException &ex)
			{
				ROS_ERROR("%s", ex.what());
				ros::Duration(1.0).sleep();
			}

			previous_pose = current_pose;
			current_pose.pose.position.x = transform.getOrigin().x();
			current_pose.pose.position.y = transform.getOrigin().y();
			quaternionTFToMsg(transform.getRotation(), current_pose.pose.orientation);


			for(int i=0;i<N;i++)
			{
				Particles[i].motion_update(current_pose, previous_pose, i);
			}

			double sum = 0;

			for(int i=0;i<N;i++)
			{
				Particles[i].measurement_update();
				sum += Particles[i].weight;
			}
			
			double w_ave = 0;
			int max_index = 0;
			
			for(int i=0;i<N;i++)
			{
				w_ave += Particles[i].weight / (double)N;
				Particles[i].weight /= sum;
				if(Particles[i].weight > Particles[max_index].weight)
					max_index = i;
			}

			if(w_slow == 0)
			{
				w_slow = w_ave;
			}

			else
			{
				w_slow += a_slow*(w_ave - w_slow);
			}

			if(w_fast == 0)
			{
				w_fast = w_ave;
			}

			else
			{
				w_fast += a_fast*(w_ave - w_fast);
			}





				

			estimated_pose = current_pose;
			geometry_msgs::PoseWithCovarianceStamped _estimated_pose;
			_estimated_pose.pose.pose = estimated_pose.pose;
			_estimated_pose.header = estimated_pose.header;
			pose_pub.publish(_estimated_pose);
			pose_array_pub.publish(poses);

			try{
				tf::StampedTransform map_transform;
				map_transform.setOrigin(tf::Vector3(estimated_pose.pose.position.x, estimated_pose.pose.position.y, 0.0));
				map_transform.setRotation(tf::Quaternion(0, 0, Get_Yaw(estimated_pose.pose.orientation), 1));
				tf::Stamped<tf::Pose> tf_stamped(map_transform.inverse(), laser.header.stamp, "base_link");
				tf::Stamped<tf::Pose> odom_to_map;
				listener.transformPose("odom", tf_stamped, odom_to_map);
				tf::Transform latest_tf = tf::Transform(tf::Quaternion(odom_to_map.getRotation()), tf::Point(odom_to_map.getOrigin()));
				temp_tf_stamped = tf::StampedTransform(latest_tf.inverse(), laser.header.stamp, "map", "odom");
				map_broadcaster.sendTransform(temp_tf_stamped);
			}

			catch(tf::TransformException ex)
			{
				std::cout << "Error!" << std::endl;
				std::cout << ex.what() << std::endl;
			}

		}
		ros::spinOnce();
		rate.sleep();
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

double calc_range(double p_x, double p_y, double yaw)
{
	int x0, x1, y0, y1;
	int dx, dy;
	int xstep, ystep;
	int x, y;
	bool flag = false;
	int err;

	x0 = (p_x - map.info.origin.position.x) / map.info.resolution;
	y0 = (p_y - map.info.origin.position.y) / map.info.resolution;

	x1 = (p_x + Max_Range * cos(yaw) - map.info.origin.position.x) / map.info.resolution;
	y1 = (p_y + Max_Range * sin(yaw) - map.info.origin.position.y) / map.info.resolution;
	
	dx = fabs(x1 - x0);
	dy = fabs(y1 - y0);

	if(dy > dx)
	{
		int temp = x1;
		x1 = x0;
		x0 = temp;

		temp = y1;
		y1 = y0;
		y0 = temp;

		flag = true;
	}
	
	dx = fabs(x1 - x0);
    dy = fabs(y1 - y0);
	x = x0;
	y = y0;

	if(x1 > x0)
		xstep = 1;
	
	else
		xstep = -1;

	if(y1 > y0)
		ystep = 1;

	else
		ystep = -1;

	

	if(flag)
	{
		if(y < 0||y > map.info.width||x < 0||x > map.info.height||map.data[y + x * map.info.width] != 0)
		{
			return sqrt(pow(x - x0, 2)+pow(y - y0, 2))*map.info.resolution;
		}
	}
	
	else
	{
        if(x < 0||x > map.info.width||y < 0||y > map.info.height||map.data[x + y * map.info.width] != 0)
        {
			return sqrt(pow(x - x0, 2)+pow(y - y0, 2))*map.info.resolution;
        }
	}
	
	while(x != x1 + xstep)
	{
		x += xstep;
		err += dy;

		if(2*err > dx)
		{
			y += ystep;
			err -= dx;
		}
	
		if(flag)
		{
			if(y < 0||y > map.info.width||x < 0||x > map.info.height||map.data[y + x * map.info.width] != 0)
			{
				return sqrt(pow(x - x0, 2)+pow(y - y0, 2))*map.info.resolution;
			}
		}

		else
		{
			if(x < 0||x > map.info.width||x < 0||y > map.info.height||map.data[x + y * map.info.width] != 0)
			{
				return sqrt(pow(x - x0, 2)+pow(y - y0, 2))*map.info.resolution;
			}
		}
	}

	return Max_Range;
}






void Particle::p_init(double x, double y, double theta)
{
    do{
		pose.pose.position.x = rand_nomal(x, x_cov);
    	pose.pose.position.y = rand_nomal(y, y_cov);
    	quaternionTFToMsg(tf::createQuaternionFromYaw(rand_nomal(theta, yaw_cov)), pose.pose.orientation);
		}while(map.data[get_index(pose.pose.position.x, pose.pose.position.y)]!=0);
}

void Particle::motion_update(geometry_msgs::PoseStamped current, geometry_msgs::PoseStamped previous, int i)
{
    double dx,dy,dyaw;
    double delta;
    double dist;  

	double yaw = Get_Yaw(pose.pose.orientation); 

    dx = current.pose.position.x - previous.pose.position.x;
    dy = current.pose.position.y - previous.pose.position.y;
    dyaw = cul_angle_diff(Get_Yaw(current.pose.orientation), Get_Yaw(previous.pose.orientation));
    dist = sqrt(dx*dx + dy*dy);

    pose.pose.position.x += dist * cos(yaw) + rand_nomal(0.0, x_cov);
    pose.pose.position.y += dist * sin(yaw) + rand_nomal(0.0, y_cov);
    quaternionTFToMsg(tf::createQuaternionFromYaw(yaw + dyaw + rand_nomal(0.0, yaw_cov)), pose.pose.orientation);
	
	geometry_msgs::Pose particle_pose;
	
	particle_pose.position.x = pose.pose.position.x;
	particle_pose.position.y = pose.pose.position.y;
	particle_pose.position.z = 0.0;
	quaternionTFToMsg(tf::createQuaternionFromYaw(Get_Yaw(pose.pose.orientation)), particle_pose.orientation);

	poses.poses[i] = particle_pose;
}

void Particle::measurement_update()
{
	double range_diff = 0;
	double p;
	double range_diff_sum = 0; 
	double angle;

	for(int i=0;i<laser.ranges.size();i+=range_count)
	{
		angle = i*laser.angle_increment - M_PI / 2;
		range_diff += laser.ranges[i] - calc_range(pose.pose.position.x, pose.pose.position.y, Get_Yaw(pose.pose.orientation)+angle);
		range_diff_sum += range_diff * range_diff;
	}

	p = exp(-1*(range_diff_sum)/(2 * sigma* sigma));

	weight *= p;
}
