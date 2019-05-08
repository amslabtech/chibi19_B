#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf/transform_listener.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

const int row = 4000;
const int column = 4000;

int grid[row][column];
float wallcost_grid[row][column];

void set_wallcost(float array[row][column]){
    int range = 35;
	float near_kl = 0;
	float far = sqrt((range/2)*(range/2)*2);
	int count = 0;
	int adjust = 3;
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
				 array[i][j] = (255.0/(far+adjust)) * (near_kl+adjust);
             }
			 else array[i][j] = 50;
         }
     }
	ROS_INFO("c = %d",count);
}

void make_heatmap(float array[row][column])
{
	const int size = row+1;
	cv::Mat_<uchar> hsv_image(row,column);
	cv::Mat_<uchar> rgb_image(row,column);

	for(int i = 0;i<row;i++){
		for(int j=0;j<column;j++){
			hsv_image(i,j) = array[i][j]; //max255
		}
	}

	//cvtColor(hsv_image, rgb_image, CV_HSV2RGB);
	
	cv::imwrite("heatmap_hsv.png",hsv_image);
	cv::imwrite("heatmap.png",rgb_image);

	cv::waitKey(0);
	cv::destroyAllWindows();
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
    set_wallcost(wallcost_grid);
	make_heatmap(wallcost_grid);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "make_heatmap");
    ros::NodeHandle map;
    ros::Subscriber map_sub = map.subscribe("map",1,map_sub_callback);
    ros::Rate loop_rate(1.0);

	ros::spin();
}

