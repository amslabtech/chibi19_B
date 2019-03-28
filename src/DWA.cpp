#include "ros/ros.h"
#include "roomba_500driver_meiji/RoombaCtrl.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "math.h"
#include <tf/tf.h>

#define max_speed 1.0
#define min_speed -0.5
#define max_yawrate 1.0
#define max_accel 0.2
#define max_dyawrate 1.0
#define v_reso 0.01
#define yawrate_reso 1.0
#define  dt 0.1f
#define predict_time 3.0
#define to_goal_cost_gain 1.0
#define speed_cost_gain 1.0
#define robot_radius 1.0


struct State{
	float x;
	float y;
	float yaw;
	float v;
	float omega;
};

struct Speed{
	float v;
	float omega;
};

struct Goal{
	float x;
	float y;
};

void motion(State roomba, Speed u){
	roomba.yaw += u.omega * dt;
	roomba.x += u.v * std::cos(roomba.yaw) * dt;
	roomba.y += u.v * std::sin(roomba.yaw) * dt;
	roomba.v = u.v;
	roomba.omega = u.omega;
};

void calc_dynamic_window(float dw[4], State roomba){
	float Vs[] = {min_speed, 
					max_speed, 
					-max_yawrate, 
					max_yawrate};

	float Vd[] = {roomba.v - max_accel * dt,
					roomba.v + max_accel * dt,
					roomba.omega - max_dyawrate * dt,
					roomba.omega + max_dyawrate * dt};

	dw[0] = std::max(Vs[0], Vd[0]);
	dw[1] = std::min(Vs[1], Vd[1]);
	dw[2] = std::min(Vs[2], Vd[2]);
	dw[3] = std::min(Vs[3], Vd[3]);
};

void calc_trajectory(std::vector<State> traj, double i, double j){
	
	State roomba = {0.0, 0.0, 0.0, 0.0, 0.0};
	Speed u ={i,j}; 
	traj.clear();

	for(double t = 0.0; t <= predict_time; t += dt){
		motion(roomba, u);
		traj.push_back(roomba);
	}
}

double calc_to_goal_cost(std::vector<State>traj, Goal goal){
	float goal_magnitude = std::sqrt(goal.x * goal.x + goal.y *goal.y);
	float traj_magnitude = std::sqrt(traj.back().x * traj.back().x + traj.back().y * traj.back().y);
	float dot_product = goal.x * traj.back().x + goal.y * traj.back().y;
	float error = dot_product / (goal_magnitude * traj_magnitude);
	float error_angle = std::acos(error);

	return to_goal_cost_gain * error_angle;
};

double calc_speed_cost(std::vector<State>traj){
	float error_speed = max_speed - traj.back().v;

	return speed_cost_gain * error_speed;
};

/*double calc_obstacle_cost(std::vector<State>traj, Goal goal){
	
	int skip_i = 2;
	int skip_j = 10;
	float min_r = std::numeric_limits<float>::infinity();
	float infinity = std::numeric_limits<float>::infinity();

	sensor_msgs::LaserScan _msg = *msg;

	const int N = int((_msg.angle_max - _msg.amgle_min) / _msg.angle_increment);
	
	float x;
	float y;
	float r_roomba;
	float angle_obstacle; 
	float range_obstacle;

	for(int i = 0;i < traj.size();i += skip_i){
		x = traj[i].x;
		y = traj[i].y;
		
		for(int j = 0;j < N;j += skip_j){
			angle_obstacle = _msg.angle_min + j*_msg.angle_increment;
			range_obstacle = _msg.ranges[j];


			if(r <= robot_radius){
				return infinity;
			}

			if(min_r >= r){
				min_r = r;
			}
		}
	}

	return 1.0 / min_r;
}*/

void calc_final_input(State roomba, Speed u, float dw[4], Goal goal){

	float min_cost = 10000.0;
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
			//ob_cost = calc_obstacle_cost(traj, goal);

			final_cost = to_goal_cost + speed_cost + ob_cost;

			if(min_cost >= final_cost){
				min_cost = final_cost;
				min_u.v = i;
				min_u.omega = j;
			}
		}
	}
};

void dwa_control(State roomba, Speed u, Goal goal,float dw[]){
	
	calc_dynamic_window(dw, roomba);
	
	calc_final_input(roomba, u, dw, goal);
};

int main(int argc, char **argv)
{
	printf("start");
	
	roomba_500driver_meiji::RoombaCtrl msg;
	msg.mode = 11;

	State roomba = {0.0, 0.0, 0.0, 0.0, 0.0};
	//[x, y, yaw, v, omega]	
	Goal goal = {10000, 10000};
	Speed u = {0.0, 0.0};
	float dw[] = {0.0, 0.0, 0.0, 0.0};
	
	while(ros::ok())
	{
	dwa_control(roomba, u, goal, dw);
	motion(roomba, u);

	//check goal
	if(sqrt(pow(roomba.x - goal.x, 2.0) + pow(roomba.y - goal.y, 2.0)) < robot_radius){
			printf("Goal!!!");
			msg.mode = 0;
			break;
		}
	}
	
	return 0;
}

