#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_listener.h>

//(21.18,19.76,0.01)

const int row = 4000;
const int column = 4000;
int grid[row][column];
const int direction = 8;
float px[row*column];
float py[row*column];

int heuristic_grid[row][column];
float cost_grid[row][column] = {-1};
int temporary_path[row*column][3];


geometry_msgs::PoseStamped path_point;
nav_msgs::Path global_path;


int delta[direction][2] = {{-1,0,},
					{-1,-1},
					{0,-1},
					{1,-1},
				   	{1,0 },
					{1,1 },
					{0,1 },
					{-1,1}};
				   
float delta_cost[direction] = {1.0,sqrtf(2.0),1.0,sqrtf(2.0),1.0,sqrtf(2.0),1.0,sqrtf(2.0)}; 

char delta_name[direction][5] = {"^","「","<","L","v","」",">","7"};

int positive_count(const int *a,const int size){
	int count = 0;
	for(int i=0;i<size;i++){
		if(a[i] > -1) count++;
	}
	return count;
}

int max_array(const int *a, int size,int &max){ //<<-return is i
	max=a[0];
	int max_i=0;
	for(int i=1;i<size;i++){
		if(max<a[i]){
			max = a[i];
			max_i = i;
		}
	}
	return max_i;
}
int min_array(const int *a, int size,int &min){ //<<positive value
	int min_i=0;
	for(int i=0;i<size;i++){
		if(a[i]>-1){
			min = a[i];
			min_i=i;
			break;
		}
		if(i==size-1) min = -1;//printf("error\n");
	}
	
	for(int i=0;i<size;i++){
		if(min>a[i] && a[i]>-1){
			min = a[i];
			min_i = i;
		}
	}
	return min_i;
}


void set_all(int array[row][column],const int setnum){
	for(int i=0;i<row;i++){
		for(int j=0;j<column;j++){
			array[i][j] = setnum;
		}
	}
}

void fset_all(float array[row][column],const float setnum){
     for(int i=0;i<row;i++){
         for(int j=0;j<column;j++){
             array[i][j] = setnum;
         }
     }
}


void set_heuristic(int array[row][column],const int goal[2]){
	array[goal[0]][goal[1]] = 0;
	for(int i=0;i<row;i++){
		for(int j=0;j<column;j++){
			array[i][j] = abs(goal[0]-i)+abs(goal[1]-j);
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

void show_cost_grid(int array[row][column]){
	for(int i=0;i<row;i++){
		for(int j=0;j<column;j++){
			if(array[i][j] == -1) printf("■ ");
			else printf("%2s",delta_name[array[i][j]]);
		}
		if(i != row-1) printf("\n");
	}
	printf("\n");
}

int count_map(){
	int ccount=0;
	for(int i=0;i<row;i++){
		for(int j=0;j<column;j++){
			if(grid[i][j] == 0) {
				//printf("% d ", grid[i][j]);
				ccount++;
			}
		}
	}
	return ccount;
}

void to_gridnum(float x,float y,int goal[2]){
	goal[0] = (100.0-y)*(20.0);
	goal[1] = (100.0+x)*(20.0);
	if(goal[0] > row-1 || goal[0] < 0 || goal[1] > column-1 || goal[1] < 0)
		printf("error.\n(to_gridnum)");
}

void to_coordnum(int gy,int gx, float &x,float &y){
     x = ((float)gx - 2000.0) / 20.0;
	 y = (2000.0 - (float)gy) / 20.0;
     
     if(x > 100 || x < -100 || y > 100 || y < -100)
         printf("error.\n(to_coordnum)");
}

int search(const int init[2],const int goal[2])
{
	static int move_history[row*column][2];
	int selected_move = 0;
	int a[direction]= {-1};
	int eight_cost[direction];
	int y = init[0];
	int x = init[1];
	int d,min_c,step,home;
	int move_direction;

	cost_grid[y][x] = 0;
	for(int f=0;f<row*column;f++){
		if(x == goal[1] && y == goal[0]){
			cost_grid[y][x] = step+1;
			temporary_path[step+1][0] = y;
			temporary_path[step+1][1] = x;
			temporary_path[step+1][2] = move_direction;
			//printf("success!!\n");
			return cost_grid[y][x];
		}
		for(int i=0;i<direction;i++){
			if(-1<(x+delta[i][1]) && column>(x+delta[i][1]) && -1<(y+delta[i][0]) && row>(y+delta[i][0])){
				eight_cost[i] = cost_grid[y+delta[i][0]][x+delta[i][1]];
				if(grid[y+delta[i][0]][x+delta[i][1]] == 0){
					if(cost_grid[y+delta[i][0]][x+delta[i][1]] < 0)
						a[i] = heuristic_grid[y+delta[i][0]][x+delta[i][1]];
					else a[i] = -1;
				}
				else a[i] = -1;
			}
			else {
				a[i] = -1;
				eight_cost[i]=-1;
			}
		}
		
		if(positive_count(a,direction)>1){
			move_history[selected_move][0]=y;
			move_history[selected_move][1]=x;
			selected_move++;
		}
		
		
		if(!positive_count(a,direction)){
			if(!selected_move){
				printf("----analysis is impossible----\n");
				return 0;
			}
			selected_move--;
			y = move_history[selected_move][0];
			x = move_history[selected_move][1];
		}
		
		else{
			home = min_array(eight_cost,direction,min_c);//from home
			step = min_c+1;
			if(step>0) temporary_path[step-1][2] = (home+4)%9;
			cost_grid[y][x] = step;
			move_direction = min_array(a,direction,d);
			temporary_path[step][0] = y;
			temporary_path[step][1] = x;
			temporary_path[step][2] = move_direction;
			x = x+delta[move_direction][1];
			y = y+delta[move_direction][0];
		}
	}
}

void mapmetadata_sub_callback(const nav_msgs::MapMetaData::ConstPtr& msg)
{
	nav_msgs::MapMetaData _msg = *msg;
	ROS_INFO("width:%6d height:%6d",_msg.width,_msg.height);
}

void map_sub_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	int count = 0;
	int ccc=0;
	nav_msgs::OccupancyGrid _msg = *msg;
	ROS_INFO("map received.");
	for(int i=row-1;i>-1;i--){
		for(int j=0;j<column;j++){
			grid[i][j] = _msg.data[count];
			count++;
			if(grid[i][j] == 0) ccc++;
		}
	}
	ROS_INFO("ccc = %d",ccc);
	global_path.header.frame_id = "map";
	fset_all(cost_grid,-1);
	int init[2] = {2000,2000};
	int goal[2];// = {1695,2030};
	to_gridnum(19.15,19.15,goal);  //(21.18,19.76,goal);

	int x,y;

	set_heuristic(heuristic_grid,goal);

	int step = search(init,goal) + 1;

    for(int i=0;i<step;i++){
         to_coordnum(temporary_path[i][0],temporary_path[i][1],px[i],py[i]);

         path_point.pose.position.x = px[i];
         path_point.pose.position.y = py[i];
         path_point.pose.position.z = 0;
         path_point.pose.orientation=tf::createQuaternionMsgFromYaw(0);

         global_path.poses.push_back(path_point);
	}

}

void click_callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
	geometry_msgs::PointStamped _msg = *msg;
	ROS_INFO("%.2f,%.2f,%.2f",_msg.point.x,_msg.point.y,_msg.point.z);
}
	

int main(int argc, char **argv)
{
	ros::init(argc, argv, "global_path_planning");
	ros::NodeHandle path;
	ros::Publisher path_pub = path.advertise<nav_msgs::Path>("chibi19_b/global_path", 1);

	ros::NodeHandle map_metadata;
	ros::NodeHandle map;
	ros::NodeHandle click;
	ros::Subscriber map_meatadata_sub = map_metadata.subscribe("map_metadata",1,mapmetadata_sub_callback);
	ros::Subscriber map_sub = map.subscribe("map",1,map_sub_callback);
	ros::Subscriber click_sub = click.subscribe("clicked_point",1,click_callback);
	ros::Rate loop_rate(0.5);

	while (ros::ok()){
         printf("count = %d\n" ,count_map());
         //msg.poses[].pose.position.x
         ROS_INFO("x,y =(%.2f, %.2f) \n", path_point.pose.position.x,path_point.pose.position.y);
         path_pub.publish(global_path);
         ros::spinOnce();
         loop_rate.sleep();
	}

}
