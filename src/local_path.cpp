/*
local path planing
DWAを用いて作成する予定

作成日：2019年3月19日
作成者：深津　蓮

更新履歴：
3/23 深津
　myenigumaのpythonコードをcppに変換

*/

#include <stdio.h>
#include <math.h>

bool show_animation = True

struct Point{
    double x; // [m]
    double y; // [m]
}

class Config(){
    // simulation parameters
public:
    double max_speed;  // [m/s]
    double min_speed;  // [m/s]
    double max_yawrate;  // [rad/s]
    double max_accel;  // [m/ss]
    double max_dyawrate;  // [rad/ss]
    double v_reso;  // [m/s]
    double yawrate_reso;  // [rad/s]
    double dt;  // [s]
    double predict_time; // [s]
    double to_goal_cost_gain;
    double speed_cost_gain;
    double robot_radius;  // [m]
    Config()
    {
        // robot parameter
        max_speed = 1.0;  // [m/s]
        min_speed = -0.5;  // [m/s]
        max_yawrate = 40.0 * math.pi / 180.0;  // [rad/s]
        max_accel = 0.2;  // [m/ss]
        max_dyawrate = 40.0 * math.pi / 180.0;  // [rad/ss]
        v_reso = 0.01;  // [m/s]
        yawrate_reso = 0.1 * math.pi / 180.0;  // [rad/s]
        dt = 0.1;  // [s]
        predict_time = 3.0;  // [s]
        to_goal_cost_gain = 1.0;
        speed_cost_gain = 1.0;
        robot_radius = 1.0;  // [m]
    };
}

Config config;

double motion(double* x, double* u, double dt)
{
    // motion model

    x[2] += u[1] * dt
    x[0] += u[0] * cos(x[2]) * dt
    x[1] += u[0] * sin(x[2]) * dt
    x[3] = u[0]
    x[4] = u[1]

    return x
}

double calc_dynamic_window(double* x)
{
    // Dynamic window from robot specification
    double Vs[4] = {config.min_speed, config.max_speed,
                    -config.max_yawrate, config.max_yawrate};

    // Dynamic window from motion model
    double Vd[4] = {x[3] - config.max_accel * config.dt,
                    x[3] + config.max_accel * config.dt,
                    x[4] - config.max_dyawrate * config.dt,
                    x[4] + config.max_dyawrate * config.dt};
    //  print(Vs, Vd)

    //  [vmin,vmax, yawrate min, yawrate max]
    double dw[4];
    for(int i=0; i<4; i++){
        if(i%2 == 0){
            if(Vs[i] > Vd[i]) dw[i] = Vs[i];
            else              dw[i] = Vd[i];
        }else{
            if(Vs[i] > Vd[i]) dw[i] = Vd[i];
            else              dw[i] = Vs[i];
        }
    }

    return &dw


double calc_trajectory(double* xinit, double v, double y)
{
    double x[5];
    double traj[5];
    for(int i=0; i<5; i++){
        x[i] = xinit[i];
        traj[i] = xinit[i];
    }
    double u[2] = { v, y};
    double time = 0.0
    while(time <= config.predict_time){
        motion(&x, &u);
        traj = np.vstack((traj, x))
        time += config.dt
    }

    return traj


double calc_final_input(double* x, double* u, double* dw, double* goal, double* ob):

    double xinit[5];
    for(int i=0; i<5; i++){
        xinit[i] = x[i];
    }
    double min_cost = 10000.0;
    double min_u[2];
    for(int i=0; i<2; i++){
        min_u[i] = u[i];
    }
    min_u[0] = 0.0
    double best_traj[5];
    for(int i=0; i<5; i++){
        best_traj[i] = x[i];
    }

    // evalucate all trajectory with sampled input in dynamic window
    for(double v=dw[0]; v<dw[1]; v+=config.v_reso){
        for(double y=dw[2]; y<dw[3]; y+=config.yawrate_reso){
            double traj = calc_trajectory(xinit, v, y)

            # calc cost
            to_goal_cost = calc_to_goal_cost(traj, goal, config)
            speed_cost = config.speed_cost_gain * \
                (config.max_speed - traj[-1, 3])
            ob_cost = calc_obstacle_cost(traj, ob, config)
            #print(ob_cost)

            final_cost = to_goal_cost + speed_cost + ob_cost

            # search minimum trajectory
            if min_cost >= final_cost:
                min_cost = final_cost
                min_u = [v, y]
                best_traj = traj

    return min_u, best_traj


def calc_obstacle_cost(traj, ob, config):
    # calc obstacle cost inf: collistion, 0:free

    skip_n = 2
    minr = float("inf")

    for ii in range(0, len(traj[:, 1]), skip_n):
        for i in range(len(ob[:, 0])):
            ox = ob[i, 0]
            oy = ob[i, 1]
            dx = traj[ii, 0] - ox
            dy = traj[ii, 1] - oy

            r = math.sqrt(dx**2 + dy**2)
            if r <= config.robot_radius:
                return float("Inf")  # collision

            if minr >= r:
                minr = r

    return 1.0 / minr  # OK


def calc_to_goal_cost(traj, goal, config):
    # calc to goal cost. It is 2D norm.

    goal_magnitude = math.sqrt(goal[0]**2 + goal[1]**2)
    traj_magnitude = math.sqrt(traj[-1, 0]**2 + traj[-1, 1]**2)
    dot_product = (goal[0]*traj[-1, 0]) + (goal[1]*traj[-1, 1])
    error = dot_product / (goal_magnitude*traj_magnitude)
    error_angle = math.acos(error)
    cost = config.to_goal_cost_gain * error_angle

    return cost


double dwa_control(double* x, double* u, double* goal, Point* ob)
{
    // Dynamic Window control

    double* dw = calc_dynamic_window(x);

    double* traj = calc_final_input(x, u, dw, goal, ob);

    return traj;
}


def plot_arrow(x, y, yaw, length=0.5, width=0.1):
    plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
              head_length=width, head_width=width)
    plt.plot(x, y)


int main()
{
    printf(" start!!");
    // initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    double x[5] = {0.0, 0.0, math.pi / 8.0, 0.0, 0.0};
    // goal position [x(m), y(m)]
    double goal[2] = {10, 10};
    // obstacles [x(m) y(m), ....]
    Point* ob = Get_obsracle();

    double u[2] = {0.0, 0.0};
    double traj[5];
    for(int i=0; i<5; i++){
        traj[i] = x[i];
    }

    for(int i=0; i<1000; i++){
        u, ltraj = dwa_control(x, u, goal, ob)

        x = motion(x, u, config.dt)
        traj = np.vstack((traj, x))  # store state history

        if show_animation:
            plt.cla()
            plt.plot(ltraj[:, 0], ltraj[:, 1], "-g")
            plt.plot(x[0], x[1], "xr")
            plt.plot(goal[0], goal[1], "xb")
            plt.plot(ob[:, 0], ob[:, 1], "ok")
            plot_arrow(x[0], x[1], x[2])
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.0001)

        # check goal
        if math.sqrt((x[0] - goal[0])**2 + (x[1] - goal[1])**2) <= config.robot_radius:
            print("Goal!!")
            break

    print("Done")
    if show_animation:
        plt.plot(traj[:, 0], traj[:, 1], "-r")
        plt.show()


if __name__ == '__main__':
    main()

