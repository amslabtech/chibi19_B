#include "ros/ros.h"
#include "roomba_500driver_meiji/RoombaCtrl.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "math.h"
#include <tf/tf.h>

#define max_speed 0.40
#define min_speed 0.1
#define max_yawrate 0.85
#define max_accel 5.0
#define max_dyawrate 2.0
#define v_reso 0.05
#define yawrate_reso 0.05
#define dt 0.1f
#define predict_time 3.0
#define to_goal_cost_gain 0.0
#define speed_cost_gain 0.0
#define robot_radius 0.19
#define roomba_v_gain 0.5
#define roomba_omega_gain 0.5

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

struct Dynamic_Window{
	double min_v;
	double max_v;
	double min_omega;
	double max_omega;
};

LaserData Ldata[N];

void motion(State& roomba, Speed u){
	roomba.yaw += u.omega * dt;
	roomba.x += u.v * std::cos(roomba.yaw) * dt;
	roomba.y += u.v * std::sin(roomba.yaw) * dt;
	roomba.v = u.v;
	roomba.omega = u.omega;
}

void calc_dynamic_window(Dynamic_Window& dw, State& roomba){
	Dynamic_Window Vs = {min_speed, 
					max_speed, 
					-max_yawrate, 
					max_yawrate};

	Dynamic_Window Vd = {roomba.v - (max_accel * dt),
					roomba.v + (max_accel * dt),
					roomba.omega - (max_dyawrate * dt),
					roomba.omega + (max_dyawrate * dt)};

	dw.min_v = std::max(Vs.min_v, Vd.min_v);
	dw.max_v = std::min(Vs.max_v, Vd.max_v);
	dw.min_omega = std::max(Vs.min_omega, Vd.min_omega);
	dw.max_omega = std::min(Vs.max_omega, Vd.max_omega);

	//ROS_INFO("[0] = %f, [1] = %f, [2] = %f, [3] = %f", dw.min_v, dw.max_v, dw.min_omega, dw.max_omega);
}

void calc_trajectory(std::vector<State>& traj, double i, double j){
	
	State roomba = {0.0, 0.0, 0.0, 0.0, 0.0};
	Speed u ={i,j}; 
	traj.clear();
	int k = 0;

	for(double t = 0.0; t <= predict_time; t += dt){
		roomba.yaw += u.omega * dt;
		roomba.x += u.v * std::cos(roomba.yaw) * dt;
		roomba.y += u.v * std::sin(roomba.yaw) * dt;
		roomba.v = u.v;
		roomba.omega = u.omega;
		traj.push_back(roomba);
		//ROS_INFO("i = %f, j = %f, traj.yaw = %f, trac.x = %f, traj.y = %f",i ,j ,traj[k].yaw, traj[k].x, traj[k].y);
		k++;
	}
}

double calc_to_goal_cost(std::vector<State>& traj, Goal goal){
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

double calc_obstacle_cost(State roomba, std::vector<State>& traj, Goal goal){
	
	int skip_k = 2;
	int skip_l = 5;
	double min_r = std::numeric_limits<double>::infinity();
	double infinity = std::numeric_limits<double>::infinity();	
	double x_traj;
	double y_traj;
	double x_roomba = roomba.x;
	double y_roomba= roomba.y;
	double r = 0;
	double angle_obstacle; 
	double range_obstacle;
	double x_obstacle;
	double y_obstacle;
	
	for(int k = 0;k < traj.size();k += skip_k){
		x_traj = traj[k].x;
		y_traj = traj[k].y;

		for(int l = 0;l < N;l += skip_l){
			
			r = 0;

			angle_obstacle = Ldata[l].angle;
			range_obstacle = Ldata[l].range;

			//ROS_INFO("l = %d angle = %f, range = %f", l, angle_obstacle, range_obstacle);
			if(range_obstacle < robot_radius){
				continue;
			}

			if(range_obstacle > 10.0){
				range_obstacle = 10.0;
			}

			x_obstacle = x_roomba + range_obstacle * std::cos(angle_obstacle);
			y_obstacle = y_roomba + range_obstacle * std::sin(angle_obstacle);
			r = std::sqrt(pow(x_obstacle - x_traj, 2.0) + pow(y_obstacle - y_traj, 2.0));
			
			//ROS_INFO("l = %d, r = %f", l, r);
			//ROS_INFO("x_roomba = %f, range_obstacle = %f, angle = %f, cos = %f", roomba.x, range_obstacle, angle_obstacle, std::cos(angle_obstacle));
			//ROS_INFO("x_od = %f, y_ob = %f", x_obstacle, y_obstacle);
			
			if(r <= robot_radius){
				return infinity;
			}

			if(min_r >= r){
				min_r = r;
			}
		}
	}
	ROS_INFO("obstacle_cost = %f", 1.0/min_r);
	return 1.0 / min_r;
}

void calc_final_input(State roomba, Speed& u, Dynamic_Window& dw, Goal goal){

	double min_cost = 10000.0;
	Speed min_u = u;
	min_u.v = 0.0;
	std::vector<State> traj;
	double to_goal_cost = 0.0;
	double speed_cost = 0.0;
	double ob_cost = 0.0;
	double final_cost = 0.0;

	for(double i = dw.min_v ; i < dw.max_v ; i += v_reso ){
		for(double j = dw.min_omega ; j < dw.max_omega ; j += yawrate_reso){
			calc_trajectory(traj, i, j);
			to_goal_cost = calc_to_goal_cost(traj, goal);
			speed_cost = calc_speed_cost(traj);
			ob_cost = calc_obstacle_cost(roomba, traj, goal);

			final_cost = to_goal_cost + speed_cost + ob_cost;

			if(min_cost >= final_cost){
				min_cost = final_cost;
				min_u.v = i;
				min_u.omega = j;
			}
		}
	}

	ROS_INFO("final cost = %f", final_cost);
	u = min_u;
}

void dwa_control(State& roomba, Speed& u, Goal goal,Dynamic_Window& dw){
	
	
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
	ros::NodeHandle roomba_odometry_sub;
	ros::NodeHandle scan_laser_sub;
	ros::Publisher ctrl_pub = roomba_ctrl_pub.advertise<roomba_500driver_meiji::RoombaCtrl>("roomba/control", 1);	
	ros::Subscriber laser_sub = scan_laser_sub.subscribe("scan", 1, lasercallback);	
	ros::Rate loop_rate(10);
	
	roomba_500driver_meiji::RoombaCtrl msg;
	nav_msgs::Odometry _msg;

	msg.mode = 11;

	State roomba = {0.0, 0.0, 0.0, 0.0, 0.0};
	//[x, y, yaw, v, omega]	
	Goal goal = {10000, 10000};
	Speed u = {0.0, 0.0};
	Dynamic_Window dw = {0.0, 0.0, 0.0, 0.0};

	while(ros::ok())
	{
	ros::spinOnce();
	
	//motion(roomba, u);
	//roomba.yaw += u.omega * dt;
	//roomba.x += u.v * std::cos(roomba.yaw) * dt;
	//roomba.y += u.v * std::sin(roomba.yaw) * dt;
	roomba.yaw = u.omega * dt;
	roomba.v = u.v;
	roomba.omega = u.omega;
	roomba.x = _msg.pose.pose.position.x;
	roomba.y = _msg.pose.pose.position.y;
	
	dwa_control(roomba, u, goal, dw);
	
	msg.cntl.linear.x = roomba_v_gain * roomba.v / max_speed;
	msg.cntl.angular.z = roomba_omega_gain * roomba.omega / max_yawrate;

	//check goal
/*	if(sqrt(pow(roomba.x - goal.x, 2.0) + pow(roomba.y - goal.y, 2.0)) < robot_radius){
			printf("Goal!!!");
			msg.cntl.linear.x = 0.0;
			B
			msg.cntl.angular.z = 0.0;
			break;
		}*/
	ctrl_pub.publish(msg);
	ROS_INFO("x = %f, z = %f", msg.cntl.linear.x, msg.cntl.angular.z);
	loop_rate.sleep();
	}
	
	return 0;
}

