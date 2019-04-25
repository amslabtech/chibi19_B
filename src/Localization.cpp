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
bool update_flag = false;

class Particle
{
public:
    Particle(void);
    void p_init(double,double, double, double, double, double);
    void motion_update(geometry_msgs::PoseStamped, geometry_msgs::PoseStamped, int);
    void measurement_update();
    
    double weight;
    
    geometry_msgs::PoseStamped pose;
    
private:
};

int N;
double init_x;
double init_y;
double init_yaw;
double init_x_cov;
double init_y_cov;
double init_yaw_cov;
double Max_Range;
double w_slow = 0.0;
double w_fast = 0.0;
double sigma = 1.0;
double a_slow;
double a_fast;
double range_count;
double a_1;
double a_2;
double a_3;
double a_4;

double motion_log = 0.0;
double yaw_log  =0.0;

double x_thresh;
double y_thresh;

double x_cov = 0.5;
double y_cov = 0.5;
double yaw_cov = 0.5;

std::vector<Particle> Particles;

double Get_Yaw(const geometry_msgs::Quaternion);

void map_callback(const nav_msgs::OccupancyGridConstPtr& msg)
{
    ROS_INFO("getting map");
	map = *msg;
    
    for(int i = 0;i < N;i++)
    {
        Particle p;
		p.p_init(init_x, init_y, init_yaw, init_x_cov, init_y_cov, init_yaw_cov);

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

double cal_angle_diff(double a, double b)
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
    ros::NodeHandle private_nh("~");
    
    ROS_INFO("Started\n");

	private_nh.getParam("N", N);
	private_nh.getParam("init_x", init_x);
	private_nh.getParam("init_y", init_y);
	private_nh.getParam("init_yaw", init_yaw);
	private_nh.getParam("init_x_cov", init_x_cov);
	private_nh.getParam("init_y_cov", init_y_cov);
	private_nh.getParam("init_yaw_cov", init_yaw_cov);
	private_nh.getParam("Max_Range", Max_Range);
	private_nh.getParam("range_count", range_count);
	private_nh.getParam("a_slow", a_slow);
	private_nh.getParam("a_fast", a_fast);
	private_nh.getParam("a_1", a_1);
	private_nh.getParam("a_2", a_2);
	private_nh.getParam("a_3", a_3);
	private_nh.getParam("a_4", a_4);
	private_nh.getParam("X_thresh", x_thresh);
	private_nh.getParam("y_thresh", y_thresh);

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
	previous_pose = current_pose;
	ros::Rate rate(10.0);

	while(ros::ok())
	{
		
		if(map_get && !laser.ranges.empty())
		{
			estimated_pose.header.frame_id = "map";
			poses.header.frame_id = "map";

			tf::StampedTransform transform;
			transform = tf::StampedTransform(tf::Transform(tf::createQuaternionFromYaw(0.0), tf::Vector3(0.0, 0.0, 0)), ros::Time::now(), "odom", "base_link");

			try
			{
				listener.waitForTransform("odom", "base_link", ros::Time(0), ros::Duration(1.0));
				listener.lookupTransform("odom", "base_link", ros::Time(0), transform);
			}

			catch(tf::TransformException &ex)
			{
				ROS_ERROR("%s", ex.what());
				ros::Duration(1.0).sleep();
			}

			current_pose.pose.position.x = transform.getOrigin().x();
			current_pose.pose.position.y = transform.getOrigin().y();
			quaternionTFToMsg(transform.getRotation(), current_pose.pose.orientation);

			if(x_cov < x_thresh || y_cov < y_thresh)
			{
				std::vector<Particle> reset_particles;
				
				x_cov = 0.3;
				y_cov = 0.3;
				yaw_cov = 0.2;
				
				for(int i=0;i<N;i++)
				{
					Particle p;

					p.p_init(estimated_pose.pose.position.x, estimated_pose.pose.position.y, Get_Yaw(estimated_pose.pose.orientation), x_cov, y_cov , yaw_cov);
					reset_particles.push_back(p);
				}

				Particles = reset_particles;
			}


			double w_sum = 0;

			for(int i=0;i<N;i++)
			{
				Particles[i].motion_update(current_pose, previous_pose, i);		//checked 2019-04-10
			}

			for(int i=0;i<N;i++)
			{
				Particles[i].measurement_update();
				w_sum += Particles[i].weight;
			}

			double w_ave = 0.0;
			int max_index = 0;

			for(int i=0;i<N;i++)
			{
				w_ave += Particles[i].weight;
				if(Particles[i].weight > Particles[max_index].weight)
					max_index = i;

				Particles[i].weight /= w_sum;
			}

			w_ave /= (double)N;

			if(w_ave == 0.0 || std::isnan(w_ave))
			{
				w_ave = 1 / (double)N;
				w_slow = w_fast = w_ave;
			}

			if(w_slow == 0.0)
			{
				w_slow = w_ave;
			}

			else
			{
				w_slow += a_slow*(w_ave - w_slow);
			}

			if(w_fast == 0.0)
			{
				w_fast = w_ave;
			}

			else
			{
				w_fast += a_fast*(w_ave - w_fast);
			}

			if(update_flag)
			{
				
				//resampling step
				int index = rand1(mt) * N;
				double beta = 0.0;
				double w;

				std::vector<Particle> New_Particles;

				w = 1 - (w_fast / w_slow);

				if(w<0)
				{
					w =0;
				}

				for(int i=0;i<N;i++)
				{
					if(w < rand1(mt))
					{
						beta += rand1(mt) * 2 * Particles[max_index].weight;
							while(beta > Particles[index].weight)
							{
								beta -= Particles[index].weight;
								index = (index+1) % N;
							}

						New_Particles.push_back(Particles[index]);
					}

					else
					{
						Particle p;
						p.p_init(estimated_pose.pose.position.x, estimated_pose.pose.position.y, Get_Yaw(estimated_pose.pose.orientation), init_x_cov,init_y_cov, init_yaw_cov);
						New_Particles.push_back(p);
					}
				}
			
				Particles = New_Particles;

				double ave_x = 0.0;
				double ave_y = 0.0;
				double ave_yaw = 0.0;

				double est_x = 0.0;
				double est_y = 0.0;
				double est_yaw = 0.0;

				sort(New_Particles.begin(), New_Particles.end(),[](const Particle& x, const Particle& y) { return x.weight > y.weight;});

 
				for(int i=0;i<1*N/4;i++)
				{
					est_x += New_Particles[i].pose.pose.position.x;
					est_y += New_Particles[i].pose.pose.position.y;
				}

				double M = 1*N/4;
				est_x /= M;
				est_y /= M;
				est_yaw = Get_Yaw(New_Particles[0].pose.pose.orientation);
				
				estimated_pose.pose.position.x = est_x;
				estimated_pose.pose.position.y = est_y;
				quaternionTFToMsg(tf::createQuaternionFromYaw(est_yaw), estimated_pose.pose.orientation);

				double new_x_cov = 0.0;
				double new_y_cov = 0.0;
				double new_yaw_cov = 0.0;

				for(int i=0;i<N;i++)
				{
					poses.poses[i] = Particles[i].pose.pose;
					ave_x += Particles[i].pose.pose.position.x;
					ave_y += Particles[i].pose.pose.position.y;
					ave_yaw += Get_Yaw(Particles[i].pose.pose.orientation);
				}

				ave_x /= N;
				ave_y /= N;
				ave_yaw /= N;

				for(int i=0;i<N;i++)
				{
					new_x_cov += (Particles[i].pose.pose.position.x - ave_x) * (Particles[i].pose.pose.position.x - ave_x);
					new_y_cov += (Particles[i].pose.pose.position.y - ave_y) * (Particles[i].pose.pose.position.y - ave_y);
					new_yaw_cov += (Get_Yaw(Particles[i].pose.pose.orientation) - ave_yaw) * (Get_Yaw(Particles[i].pose.pose.orientation) - ave_yaw);
				}

				x_cov = sqrt(new_x_cov/N);
				y_cov = sqrt(new_y_cov/N);
				yaw_cov = sqrt(new_yaw_cov/N);
			}

			update_flag = false;
		}

			geometry_msgs::PoseWithCovarianceStamped _estimated_pose;
			_estimated_pose.pose.pose = estimated_pose.pose;
			_estimated_pose.header = estimated_pose.header;
			pose_pub.publish(_estimated_pose);
			pose_array_pub.publish(poses);

			try
			{
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

		ros::spinOnce();
		rate.sleep();

		previous_pose = current_pose;
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
	int err = 0;

	x0 = (p_x - map.info.origin.position.x) / map.info.resolution;
	y0 = (p_y - map.info.origin.position.y) / map.info.resolution;

	x1 = (p_x + Max_Range * cos(yaw) - map.info.origin.position.x) / map.info.resolution;
	y1 = (p_y + Max_Range * sin(yaw) - map.info.origin.position.y) / map.info.resolution;
	
	if(fabs(x1 - x0) < fabs(y1 - y0))
	{
		flag = true;
	}

	if(flag)
	{
		int temp = x1;
		x1 = y1;
		y1 = temp;

		temp = x0;
		x0 = y0;
		y0 = temp;

	}
	
	dx = abs(x1 - x0);
    dy = abs(y1 - y0);

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

void Particle::p_init(double x, double y, double theta, double cov_1, double cov_2, double cov_3)
{
    do{
		pose.pose.position.x = rand_nomal(x, cov_1);
    	pose.pose.position.y = rand_nomal(y, cov_2);
    	quaternionTFToMsg(tf::createQuaternionFromYaw(rand_nomal(theta, cov_3)), pose.pose.orientation);
		}while(map.data[get_index(pose.pose.position.x, pose.pose.position.y)]!=0);
}

void Particle::motion_update(geometry_msgs::PoseStamped current, geometry_msgs::PoseStamped previous, int i)
{
    double dx,dy,dyaw;
    double delta;
	double yaw = Get_Yaw(pose.pose.orientation);

	double del_rot1, del_rot2, del_trans;
	double del_rot1_hat, del_rot2_hat, del_trans_hat;
	double del_rot1_noise, del_rot2_noise;
    dx = current.pose.position.x - previous.pose.position.x;
    dy = current.pose.position.y - previous.pose.position.y;
    dyaw = cal_angle_diff(Get_Yaw(current.pose.orientation), Get_Yaw(previous.pose.orientation));

	motion_log +=sqrt(dx*dx + dy*dy);
	yaw_log += fabs(dyaw);

	if(motion_log > 0.2 || yaw_log >0.15)
	{
		update_flag = true;
		motion_log = 0.0;
		yaw_log = 0.0;
	}

	if(sqrt(dx*dx + dy*dy)<0.01)
		del_rot1 = 0;

	else
		del_rot1 = dyaw;

	del_trans = sqrt(dx*dx + dy*dy);
	del_rot2 = cal_angle_diff(dyaw, del_rot1);
	
	del_rot1_noise = std::min(fabs(cal_angle_diff(del_rot1, 0.0)),fabs(cal_angle_diff(del_rot1, M_PI)));
	del_rot2_noise = std::min(fabs(cal_angle_diff(del_rot2, 0.0)),fabs(cal_angle_diff(del_rot2, M_PI)));
	
	del_rot1_hat = cal_angle_diff(del_rot1, rand_nomal(0.0, a_1*del_rot1_noise*del_rot1_noise - a_2*del_trans*del_trans));
	del_trans_hat = del_trans - rand_nomal(0.0, a_3*del_trans*del_trans + a_4*del_rot1_noise*del_rot1_noise + a_4*del_rot2_noise*del_rot2_noise);
	del_rot2_hat = cal_angle_diff(del_rot2, rand_nomal(0.0, a_1*del_rot2_noise*del_rot2_noise - a_2*del_trans*del_trans));
	
    pose.pose.position.x += del_trans_hat * cos(yaw + del_rot1_hat);
    pose.pose.position.y += del_trans_hat * sin(yaw + del_rot1_hat);
    quaternionTFToMsg(tf::createQuaternionFromYaw(yaw + del_rot1_hat + del_rot2_hat), pose.pose.orientation);
	
}

void Particle::measurement_update()
{
	double range_diff = 0;
	double p = 0.0;
	double map_range; 
	double angle;

	double z_short = 0.1;
	double z_hit = 0.7;
	double z_max = 0.1;
	double z_random = 0.1;

	double lambda_short = 0.2;
	
	for(int i=0;i<laser.ranges.size();i+=range_count)
	{
		angle = i*laser.angle_increment + laser.angle_min;
		map_range = calc_range(pose.pose.position.x, pose.pose.position.y, Get_Yaw(pose.pose.orientation)+angle);
		range_diff = laser.ranges[i] - map_range;
	
		if(laser.ranges[i] < Max_Range)
		{
			p += z_hit * exp(-1*(range_diff * range_diff)/(2 * sigma* sigma));
		}

		if(range_diff < 0)
		{
			p += lambda_short * z_short * exp(-lambda_short * laser.ranges[i]);
		}

		if(laser.ranges[i] >= Max_Range)
		{
			p += z_max * 1.0;
		}

		if(laser.ranges[i] < Max_Range)
		{
			p += z_random * 1.0 / Max_Range;
		}
	}

	weight = p;
}
