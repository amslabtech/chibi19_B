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
    int range = 20;
	float near_kl = 0;
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
	ROS_INFO("c = %d",count);
}


void ColorScaleBCGYR( double in_value, int &r,int &g,int &b)
{
    // 0.0～1.0 の範囲の値をサーモグラフィみたいな色にする
    double  value = in_value;
    double  tmp_val = cos( 4 * M_PI * value );
    int     col_val = (int)( ( -tmp_val / 2 + 0.5 ) * 255 );
         if ( value >= ( 4.0 / 4.0 ) ) { r = 255;     g = 0;       b = 0;       }   // 赤
    else if ( value >= ( 3.0 / 4.0 ) ) { r = 255;     g = col_val; b = 0;       }   // 黄～赤
    else if ( value >= ( 2.0 / 4.0 ) ) { r = col_val; g = 255;     b = 0;       }   // 緑～黄
    else if ( value >= ( 1.0 / 4.0 ) ) { r = 0;       g = 255;     b = col_val; }   // 水～緑
    else if ( value >= ( 0.0 / 4.0 ) ) { r = 0;       g = col_val; b = 255;     }   // 青～水
    else {                               r = 0;       g = 0;       b = 255;     }   // 青
}

void make_heatmap(float array[row][column])
{
	int b,g,r;
	const int size = row+1;
	cv::Mat heatmap_img(row,column,CV_8UC3);
	cv::Mat color_img(100,600,CV_8UC3);

	for(int i = 0;i<row;i++){
		for(int j=0;j<column;j++){
			if(array[i][j] > 5) heatmap_img.at<cv::Vec3b>(i,j) = cv::Vec3b(100,100,100); 
			else{
				ColorScaleBCGYR(array[i][j],r,g,b);
				heatmap_img.at<cv::Vec3b>(i,j) = cv::Vec3b(b,g,r); //bgr
			}
		}
	}

	for(int i = 0;i<100;i++){
		for(int j=0;j<600;j++){
			ColorScaleBCGYR((float)j/599.0,r,g,b);
            color_img.at<cv::Vec3b>(i,j) = cv::Vec3b(b,g,r); //bgr
		}
	}


	cv::imwrite("heatmap_hsv.png",heatmap_img);
	cv::imwrite("color_sample.png",color_img);
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

