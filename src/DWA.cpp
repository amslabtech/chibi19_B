#include "ros/ros.h"
#include "roomba_500driver_meiji/RoombaCtrl.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "math.h"
#include <tf/tf.h>

#define max_speed 0.40
#define min_speed -0.5
#define max_yawrate 1.0
#define max_accel 0.2
#define max_dyawrate 1.0
#define v_reso 0.05
#define yawrate_reso 1.0
#define dt 0.1f
#define predict_time 3.0
#define to_goal_cost_gain 0.0
#define speed_cost_gain 0.0
#define robot_radius 0.19
#define roomba_v_gain 2.5
#define roomba_omega_gain 1.25

const int N = 720;//(_msg.angle_max - _msg.angle_max) / _msg.angle_increment;

struct State{
	double x;
	double y;
	double yaw;
	double v;
	double omega;
};

struct Speed{
	double v;
	double omega;
};

struct Goal{
	double x;
	double y;
};

struct LaserData{
	double angle;
	double range;
};

LaserData Ldata[N];

void motion(State &roomba, Speed u){
	roomba.yaw += u.omega * dt;
	roomba.x += u.v * std::cos(roomba.yaw) * dt;
	roomba.y += u.v * std::sin(roomba.yaw) * dt;
	roomba.v = u.v;
	roomba.omega = u.omega;
}

void calc_dynamic_window(double dw[4], State roomba){
	double Vs[] = {min_speed, 
					max_speed, 
					-max_yawrate, 
					max_yawrate};

	double Vd[] = {roomba.v - max_accel * dt,
					roomba.v + max_accel * dt,
					roomba.omega - max_dyawrate * dt,
					roomba.omega + max_dyawrate * dt};

	dw[0] = std::max(Vs[0], Vd[0]);
	dw[1] = std::min(Vs[1], Vd[1]);
	dw[2] = std::min(Vs[2], Vd[2]);
	dw[3] = std::min(Vs[3], Vd[3]);
}

void calc_trajectory(std::vector<State> &traj, double i, double j){
	
	State roomba = {0.0, 0.0, 0.0, 0.0, 0.0};
	Speed u ={i,j}; 
	traj.clear();

	for(double t = 0.0; t <= predict_time; t += dt){
		motion(roomba, u);
		traj.push_back(roomba);
	}
}

double calc_to_goal_cost(std::vector<State> &traj, Goal goal){
	double goal_magnitude = std::sqrt(goal.x * goal.x + goal.y *goal.y);
	double traj_magnitude = std::sqrt(traj.back().x * traj.back().x + traj.back().y * traj.back().y);
	double dot_product = goal.x * traj.back().x + goal.y * traj.back().y;
	double error = dot_product / (goal_magnitude * traj_magnitude);
	double error_angle = std::acos(error);

	return to_goal_cost_gain * error_angle;
}

double calc_speed_cost(std::vector<State> traj){
	double error_speed = max_speed - traj.back().v;

	return speed_cost_gain * error_speed;
}

double calc_obstacle_cost(State roomba, std::vector<State> &traj, Goal goal){
	
	int skip_i = 2;
	int skip_j = 20;
	double min_r = std::numeric_limits<double>::infinity();
	double infinity = std::numeric_limits<double>::infinity();	
	double x_traj;
	double y_traj;
	double x_roomba = roomba.x;
	double y_roomba= roomba.y;
	double r;
	double angle_obstacle; 
	double range_obstacle;
	double x_obstacle;
	double y_obstacle;
	
	for(int i = 0;i < traj.size();i += skip_i){
		x_traj = traj[i].x;
		y_traj = traj[i].y;
		
		for(int j = 0;j < N;j += skip_j){
			
			ROS_INFO("j = %d\n", j);

			angle_obstacle = Ldata[j].angle;
			range_obstacle = Ldata[j].range;
			x_obstacle = x_roomba + range_obstacle * std::cos(angle_obstacle);
			y_obstacle = y_roomba + range_obstacle * std::sin(angle_obstacle);
			r = std::sqrt(pow(x_obstacle - x_traj, 2.0) + pow(y_obstacle - y_traj, 2.0));

			if(r <= robot_radius){
				return infinity;
			}

			ROS_INFO("r = %d\n", r);

			if(min_r >= r){
				min_r = r;
			}
		}
	}

	return 1.0 / min_r;
}

void calc_final_input(State roomba, Speed &u, double dw[4], Goal goal){

	double min_cost = 10000.0;
	Speed min_u = u;
	min_u.v = 0.0;
	std::vector<State> traj;
	double to_goal_cost;
	double speed_cost;
	double ob_cost;
	double final_cost;

	for(double i = dw[0] ; i < dw[1] ; i += v_reso ){
		for(double j = dw[2] ; j < dw[3] ; j += yawrate_reso){
			calc_trajectory(traj, i, j);
			to_goal_cost = calc_to_goal_cost(traj, goal);
			speed_cost = calc_speed_cost(traj);
			ROS_INFO("1\n");
			ob_cost = calc_obstacle_cost(roomba, traj, goal);

			final_cost = to_goal_cost + speed_cost + ob_cost;

			if(min_cost >= final_cost){
				min_cost = final_cost;
				min_u.v = i;
				min_u.omega = j;
			}
		}
	}

	u = min_u;
}

void dwa_control(State &roomba, Speed &u, Goal goal,double dw[]){
	
	
	calc_dynamic_window(dw, roomba);
	
	calc_final_input(roomba, u, dw, goal);
}

void lasercallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    sensor_msgs::LaserScan _msg = *msg;

    for(int i=0; i<N; i++){
        Ldata[i].angle = _msg.angle_min + i*_msg.angle_increment;
        Ldata[i].range = _msg.ranges[i];
    }
}

int main(int argc, char **argv)
{	
	ros::init(argc, argv, "dwa");	
	ros::NodeHandle roomba_ctrl_pub;
	ros::NodeHandle scan_laser_sub;
	ros::Publisher ctrl_pub = roomba_ctrl_pub.advertise<roomba_500driver_meiji::RoombaCtrl>("roomba/control", 1);	
	ros::Subscriber laser_sub = scan_laser_sub.subscribe("scan", 1, lasercallback);	
	ros::Rate loop_rate(10);
	
	roomba_500driver_meiji::RoombaCtrl msg;
	msg.mode = 11;

	State roomba = {0.0, 0.0, 0.0, 0.0, 0.0};
	//[x, y, yaw, v, omega]	
	Goal goal = {10000, 10000};
	Speed u = {0.0, 0.0};
	double dw[] = {0.0, 0.0, 0.0, 0.0};

	while(ros::ok())
	{
	ros::spinOnce();
	
	ROS_INFO("loop");

	dwa_control(roomba, u, goal, dw);
	//motion(roomba, u);
	roomba.yaw += u.omega * dt;
	roomba.x += u.v * std::cos(roomba.yaw) * dt;
	roomba.y += u.v * std::sin(roomba.yaw) * dt;
	roomba.v = u.v;
	roomba.omega = u.omega;

	msg.cntl.linear.x = roomba.v * roomba_v_gain;
	msg.cntl.angular.z = roomba.omega * roomba_omega_gain;

	//check goal
/*	if(sqrt(pow(roomba.x - goal.x, 2.0) + pow(roomba.y - goal.y, 2.0)) < robot_radius){
			printf("Goal!!!");
			msg.cntl.linear.x = 0.0;
			msg.cntl.angular.z = 0.0;
			break;
		}*/
	ctrl_pub.publish(msg);
	ROS_INFO("x = %f, z = %f\n", msg.cntl.linear.x, msg.cntl.angular.z);
	loop_rate.sleep();
	}
	
	return 0;
}

