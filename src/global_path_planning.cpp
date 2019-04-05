#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_listener.h>

const int row = 4000;
const int column = 4000;
int search_count=0;

const int d = 8;//houkousyurui

int delta[d][2] = {{-1,0,},
                     {-1,-1},
                     {0,-1},
                     {1,-1},
                     {1,0 },
                     {1,1 },
                     {0,1 },
                     {-1,1}};
float delta_cost[d] = {1.0,sqrtf(2.0),1.0,sqrtf(2.0),1.0,sqrtf(2.0 ),1.0,sqrtf(2.0)}; 

int grid[row][column];
int open_grid[row][column];
int close_grid[row][column];
int heuristic[row][column];

nav_msgs::Path global_path;
nav_msgs::Path connect_path;

struct Point
{
	float cost;
	float gvalue;
	int x;
	int y;
	int direction;

	bool operator<(const Point &another) const
	{
        return cost > another.cost;//年齢を比較
    };
};

void set_all(int array[row][column],const int setnum = -1){
    for(int i=0;i<row;i++){
        for(int j=0;j<column;j++){
            array[i][j] = setnum;
        }
    }
}

void show_array(int array[row][column]){
	printf("[[");
	for(int i=0;i<row;i++){
		for(int j=0;j<column;j++){
            printf("%2d",array[i][j]);
            if(j != column-1) printf(",");
        }
        printf("]");
        if(i != row-1) printf("\n [");
    }
    printf("]\n");
}

void show_farray(float array[row][column]){
     printf("[[");
     for(int i=0;i<row;i++){
         for(int j=0;j<column;j++){
             printf("%4.1f",array[i][j]);
             if(j != column-1) printf(",");
         }
         printf("]");
         if(i != row-1) printf("\n [");
     }
     printf("]\n");
}

void set_heuristic(int array[row][column],const int goal[2]){
    array[goal[0]][goal[1]] = 0;
    for(int i=0;i<row;i++){
        for(int j=0;j<column;j++){
            array[i][j] = abs(goal[0]-i)+abs(goal[1]-j);
        }
    }
}


Point make_Point(float cost,float gvalue,int x,int y,int direction)
{
	Point p;
	p.cost = cost;
	p.gvalue = gvalue;
	p.x = x;
	p.y = y;
	p.direction = direction;
	
	return p;
}

void get_param(Point p,float &cost,float &gvalue,int &x,int &y,int &direction)
{
	cost = p.cost;
	gvalue = p.gvalue;
	x = p.x;
	y = p.y;
	direction = p.direction;
}

int search(const int init[2],const int goal[2])
{
	int x,y,x2,y2,direction,direction2;
	float cost,cost2,gvalue,gvalue2;
	std::vector<Point> open_Point;

	cost = 0;
	gvalue = 0;
	x = init[1];
	y = init[0];
	direction = 0; 

	open_Point.push_back(make_Point(cost,gvalue,x,y,direction));

	set_heuristic(heuristic,goal);
	set_all(open_grid);
	set_all(close_grid);
	open_grid[y][x] = 10;

	for(int step=0; step<row*column; step++){
		if (open_Point.size() < 1){
			//printf("-----------mis----------\n");
			ROS_INFO("miss");
			break;
		}
		std::sort(open_Point.begin(),open_Point.end());
		get_param(open_Point[open_Point.size()-1],cost,gvalue,x,y,direction);
		open_Point.pop_back();
		close_grid[y][x] = 1;
		open_grid[y][x] = (direction+4)%8;
		for(int i=0;i<d;i++){
			direction2 = (direction+4)%8;
			if(i != direction2){
				x2 = x + delta[i][1];
				y2 = y + delta[i][0];
				if(x2<column && x2>-1 && y2<row && y2>-1){
					if(close_grid[y2][x2] < 0 && grid[y2][x2] == 0){
						gvalue2 = gvalue + delta_cost[i];
						cost2 = gvalue2 + heuristic[y2][x2];
						//open_grid[y2][x2] = (i+4)%8;
						open_Point.push_back(make_Point(cost2,gvalue2,x2,y2,i));
						if(heuristic[y2][x2] == 0){
							open_grid[y2][x2] = (i+4)%8;
							i=d;
							step = row*column;
							ROS_INFO("success");
							break;
						}
					}
				}
			}
		}
	}
}

void to_gridnum(float x,float y,int goal[2]){
    goal[0] = (100.0-y)*(20.0);
    goal[1] = (100.0+x)*(20.0);
    if(goal[0] > row-1 || goal[0] < 0 || goal[1] > column-1 || goal[1] < 0 )
        printf("error.\n(to_gridnum)");
}

void to_coordnum(int gx,int gy, float &x,float &y){
     x = ((float)gx - 2000.0) / 20.0;
     y = (2000.0 - (float)gy) / 20.0;

     if(x > 100 || x < -100 || y > 100 || y < -100)
         printf("error.\n(to_coordnum)");
}


void get_path(int goal[2])
{
	int gx = goal[1];
	int gy = goal[0];
	int direction;
	float x,y;
	geometry_msgs::PoseStamped path_point;
	connect_path.poses.clear();

	for(int i = 0;i<row*column;i++){
		to_coordnum(gx,gy,x,y);

		path_point.pose.position.x = x;
        path_point.pose.position.y = y;
        path_point.pose.position.z = 0;
        path_point.pose.orientation=tf::createQuaternionMsgFromYaw(0);

		connect_path.poses.push_back(path_point);

		direction = open_grid[gy][gx];
		if(direction < d && direction > -1){
			gx = gx + delta[direction][1];
			gy = gy + delta[direction][0];
		}
		else break;
	}
	std::reverse(connect_path.poses.begin(),connect_path.poses.end());
	global_path.poses.insert(global_path.poses.end(),connect_path.poses.begin(),connect_path.poses.end());
	ROS_INFO("get finish");
}

void map_sub_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{   
	int count = 0;
    nav_msgs::OccupancyGrid _msg = *msg;
    ROS_INFO("map received.");
    for(int i=row-1;i>-1;i--){
        for(int j=0;j<column;j++){
            grid[i][j] = _msg.data[count];
			count++;
        }
    }
    global_path.header.frame_id = "map";
    int init[2];
    int goal[2];// = {1695,2030};
    
    to_gridnum(0,0,init);  //(21.18,19.76,goal);
    to_gridnum(16.08,-0.24,goal);  //(21.18,19.76,goal);

	search(init,goal);
	get_path(goal);
}


void click_callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
	int init[2];
	int goal[2];
	float x = global_path.poses[global_path.poses.size()-1].pose.position.x;
	float y = global_path.poses[global_path.poses.size()-1].pose.position.y;

    geometry_msgs::PointStamped _msg = *msg;
    ROS_INFO("%.2f,%.2f,%.2f",_msg.point.x,_msg.point.y,_msg.point.z);
    ROS_INFO("%.2f,%.2f",x,y);

	to_gridnum(x,y,init);
	to_gridnum(_msg.point.x,_msg.point.y,goal);
	search(init,goal);
	get_path(goal);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "global_path_planning");
	ros::NodeHandle path;
    ros::NodeHandle map;
    ros::NodeHandle click;
    ros::Subscriber map_sub = map.subscribe("map",1,map_sub_callback);
    ros::Subscriber click_sub = click.subscribe("clicked_point",1,click_callback);
    ros::Publisher path_pub = path.advertise<nav_msgs::Path>("chibi19_b/global_path", 1);
    ros::Rate loop_rate(0.2);

	while (ros::ok()){
          path_pub.publish(global_path);
		  ROS_INFO("path is published");
          ros::spinOnce();
          loop_rate.sleep();
    }
}
