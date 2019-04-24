#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf/transform_listener.h>
//ac

const int row = 4000;
const int column = 4000;
int search_count=0;
int sg = 0;

const int d = 8;//houkousyurui

int delta[d][2] = {{-1,0,},
                     {-1,-1},
                     {0,-1},
                     {1,-1},
                     {1,0 },
                     {1,1 },
                     {0,1 },
                     {-1,1}};

//int delta[d][2] = {{-1,0,},
//                   {-2,-1},
//                   {-1,-1},
//                   {-1,-2},
//                   {0,-1 },
//                   {1,-2 },
//                   {1,-1 },
//                   {2,-1 },
//                   {1,0  },
//                   {2,1  },
//                   {1,1  },
//                   {1,2  },
//                   {0,1  },
//                   {-1,2 },
//                   {-1,1 },
//                   {-2,1 }};
//
float delta_cost[d] = {1.0,sqrtf(2.0),1.0,sqrtf(2.0),1.0,sqrtf(2.0 ),1.0,sqrtf(2.0)}; 
//

//float delta_cost[d] = {1.0,sqrtf(5.0),sqrtf(2.0),sqrtf(5.0),1.0,sqrtf(5.0),sqrtf(2.0),sqrtf(5.0),1.0,sqrtf(5.0),sqrtf(2.0 ),sqrtf(5.0),1.0,sqrtf(5.0),sqrtf(2.0),sqrtf(5.0)}; 

int grid[row][column];
int open_grid[row][column];
int close_grid[row][column];
float heuristic[row][column];
float wallcost_grid[row][column];

nav_msgs::Path global_path;
nav_msgs::Path connect_path;
geometry_msgs::PointStamped target_point;

struct Point
{
	float cost;
	float gvalue;
	int x;
	int y;
	int direction;

	bool operator<(const Point &another) const
	{
        return cost > another.cost;
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

void set_heuristic(float array[row][column],const int goal[2]){
    for(int i=0;i<row;i++){
        for(int j=0;j<column;j++){
            array[i][j] =sqrt(  pow((goal[0]-i),2) + pow((goal[1]-j),2)) ;
            //array[i][j] = abs(goal[0]-i) + abs(goal[1]-j) ;
        }
    }
}

void set_wallcost(float array[row][column]){ //0~1.0
    int range = 20;
    float near_kl;
    float far = sqrt((range/2)*(range/2)*2);
    int count = 0;
    int adjust = 0;
    float max_value = 1.0;
    for(int i=0;i<row;i++){
         for(int j=0;j<column;j++){
             if(grid[i][j] == 0){
                 near_kl = far;

                 for(int k=0;k<range+1;k++){
                     for(int l=0;l<range+1;l++){
                        if(grid[i-range/2+k][j-range/2+l] != 0){
                            if( sqrt(pow(abs(k-range/2),2)+pow(abs(l-range/2),2)) < near_kl){
                                near_kl = sqrt(pow(abs(k-range/2),2)+pow(abs(l-range/2),2));
                                count++;
                            }
                        }
                     }
                 }
                 array[i][j] = max_value - (max_value/(far+adjust)) * (near_kl+adjust);
             }
             else array[i][j] = 10.0;
         }
     }
}

//void set_wallcost(float array[row][column]){
//	int range = 12;
//	for(int i=0;i<row;i++){
//         for(int j=0;j<column;j++){
//			 if(grid[i][j] == 100){
//				 for(int k=0;k<range;k++){
//					 for(int l=0;l<range-k;l++){
//						array[i+k][j+l] += 2.0;
//						array[i-k][j-l] += 2.0;
//					 }
//				 }
//			 }
//         }
//     }
//}


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

	float a = 1.0;
	float b = 1.1;
	float c = 10.0;

	open_Point.push_back(make_Point(cost,gvalue,x,y,direction));

	set_heuristic(heuristic,goal);
	set_all(open_grid);
	set_all(close_grid);
	open_grid[y][x] = 10;

	for(int step=0; step<row*column; step++){
		if(step == 50000){
			printf("\n<<<<<<<<<<<<timeover>>>>>>>>>>>>>>>>\n");
			break;
		}
		if (open_Point.size() < 1){
			ROS_INFO("analysis impossible");
			break;
		}
		std::sort(open_Point.begin(),open_Point.end());
		get_param(open_Point.back(),cost,gvalue,x,y,direction);
		open_Point.pop_back();
		close_grid[y][x] = 1;
		open_grid[y][x] = (direction+d/2)%d;
		printf("\rclose is done.(%6d,%6d) H_grid=%5.1f cost=%5.1f (%3.1lf[parsent])",x,y,heuristic[y][x],cost,(double)step);
		for(int i=0;i<d;i++){
			direction2 = (direction+d/2)%d;
			if(i != direction2){
				x2 = x + delta[i][1];
				y2 = y + delta[i][0];

				if(x2<column && x2>-1 && y2<row && y2>-1){
					if(close_grid[y2][x2] < 0 && grid[y2][x2] == 0){
						gvalue2 = gvalue + delta_cost[i];
						cost2 = a*gvalue2 + b*heuristic[y2][x2] + c*wallcost_grid[y2][x2];
						//open_grid[y2][x2] = (i+4)%8;
						open_Point.push_back(make_Point(cost2,gvalue2,x2,y2,i));
						if(heuristic[y2][x2] == 0.0 ){
							open_grid[y2][x2] = (i+d/2)%d;
							i=d;
							step = row*column;
							printf("\n");
							ROS_INFO("success!!!!");
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
	ROS_INFO("get path (size = %lu)\n",global_path.poses.size());
}

void set_randmark(const float x,const float y)
{
	int init[2];
	int goal[2];
	float ini_x = global_path.poses.back().pose.position.x;
	float ini_y = global_path.poses.back().pose.position.y;

    ROS_INFO("randmark = (%.2f,%.2f)",x,y);

	to_gridnum(ini_x,ini_y,init);
	to_gridnum(x,y,goal);
	if(grid[goal[0]][goal[1]] == 0){
		search(init,goal);
		get_path(goal);
	}
	else ROS_INFO("goal is not free space.");
}

void click_callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    geometry_msgs::PointStamped _msg = *msg;

	int t_dis = 60;
	int target_i,min_i;
	float x = _msg.point.x;
	float y = _msg.point.y;
	float dis,min;
	int start_i = 0;
	int quater_size = global_path.poses.size()/4;

    ROS_INFO("click point =(%.2f,%.2f)",x,y);
	
     if(y > 13.0) sg = 1;
	
	if(sg == 1) start_i = quater_size;
	min = pow((x-global_path.poses[start_i].pose.position.x),2.0)+pow((y-global_path.poses[start_i].pose.position.y),2.0);
	min_i = start_i;

	for(int i=0+start_i;i<global_path.poses.size()-(quater_size-start_i);i++){
		dis = pow((x-global_path.poses[i].pose.position.x),2.0)+pow((y-global_path.poses[i].pose.position.y),2.0);
		if(dis < min){
			min = dis;
			min_i = i;
		}
	} 
	if((min_i+t_dis) > global_path.poses.size()-1) 
		target_i = global_path.poses.size();
	else 
		target_i = min_i+t_dis;
	ROS_INFO("min_i = %d target_i = %d sg=%d",min_i,target_i,sg);

	target_point.point.x = global_path.poses[target_i].pose.position.x;
	target_point.point.y = global_path.poses[target_i].pose.position.y;
	target_point.point.z = 0;
	target_point.header.frame_id = "map";
	ROS_INFO("target = (%.2f,%.2f)",target_point.point.x ,target_point.point.y);
}


void localization_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    geometry_msgs::PoseWithCovarianceStamped _msg = *msg;

	int t_dis = 45; // t_dis*0.05[m]far is goal
	int target_i,min_i; 
	float x = _msg.pose.pose.position.x;
	float y = _msg.pose.pose.position.y;
	float dis,min;
	int start_i = 0;
	int quater_size = global_path.poses.size()/4;

	if(y > 13.0) sg = 1;

	if(sg == 1) start_i = quater_size;
	min = pow((x-global_path.poses[start_i].pose.position.x),2.0)+pow((y-global_path.poses[start_i].pose.position.y),2.0);
	min_i = start_i;

	for(int i=0+start_i;i<global_path.poses.size()-(quater_size-start_i);i++){
		dis = pow((x-global_path.poses[i].pose.position.x),2.0)+pow((y-global_path.poses[i].pose.position.y),2.0);
		if(dis < min){
			min = dis;
			min_i = i;
		}
	}
	if((min_i+t_dis) > global_path.poses.size()-1) target_i = global_path.poses.size();
	else target_i = min_i+t_dis;

	target_point.point.x = global_path.poses[target_i].pose.position.x;
	target_point.point.y = global_path.poses[target_i].pose.position.y;
	target_point.point.z = 0;
	target_point.header.frame_id = "map";
}

void set_init(const float x,const float y)
 {
 	geometry_msgs::PoseStamped path_point;
 	path_point.pose.position.x = x;
 	path_point.pose.position.y = y;
 	path_point.pose.position.z = 0;
 	path_point.pose.orientation=tf::createQuaternionMsgFromYaw(0);
 	global_path.poses.push_back(path_point);
 }
 
 void round_DF1()
 {	
 	set_init(0.0,0.0);

  	set_randmark(16.19,-0.18);
  	set_randmark(16.0,14.17);
  	set_randmark(-17.34,14.25);
 	set_randmark(-17.15,-0.10);
 
// 	set_randmark(16.15,3.70);
// 	set_randmark(12.55,17.27);
// 	set_randmark(-19.59,8.84);
// 	set_randmark(-16.00,-4.70);

 	set_randmark(0.0,0.0);
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
	set_wallcost(wallcost_grid);
	round_DF1();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "global_path_planning");
	ros::NodeHandle path;
    ros::NodeHandle map;
    ros::NodeHandle click;
    ros::NodeHandle localization;
	ros::NodeHandle target;
    ros::Subscriber map_sub = map.subscribe("map",1,map_sub_callback);
    ros::Subscriber click_sub = click.subscribe("clicked_point",1,click_callback);
    ros::Subscriber localization_sub = localization.subscribe("/chibi19/estimated_pose",1,localization_callback);
    ros::Publisher path_pub = path.advertise<nav_msgs::Path>("chibi19_b/global_path", 1);
	ros::Publisher target_pub = target.advertise<geometry_msgs::PointStamped>("chibi19_b/target", 1);
    ros::Rate loop_rate(5.0);

	//round_DF1();

	while (ros::ok()){
          path_pub.publish(global_path);
		  target_pub.publish(target_point);
		  ROS_INFO("path is published (point %lu)",global_path.poses.size());
          ros::spinOnce();
          loop_rate.sleep();
    }
}

