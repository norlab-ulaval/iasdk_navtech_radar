
/*
Copyright 2016 Navtech Radar Limited
This file is part of iasdk which is released under The MIT License (MIT).
See file LICENSE.txt in project root or go to https://opensource.org/licenses/MIT
for full license details.
*/


/* 
Node Description: 
Example Application: Moving Platform Data Processor + Visualizer
This app will subscribe to the raw data and give a user-configurable way to process this data.
Functionality can be switched on/off using the dynamic reconfigure rqt plugin.
Functionality available:
Thresholded pointcloud of raw data
Image topic using object detection algorithms 
Grid lines
*/

#define AZIMUTH_AVERAGING true

#include "radarclient.h"
#include "nav_ross/nav_msg.h"

#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <cstdint>
#include <functional>
#include <chrono>
#include <thread>
#include <cmath>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <dynamic_reconfigure/server.h>
#include <dynamic_paramConfig.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float32.h>

#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/LaserScan.h>

#ifdef _WIN32
#include <winsock2.h>
#pragma comment(lib, "Ws2_32.lib")
#pragma comment(lib, "Rpcrt4.lib")
#pragma comment(lib, "Iphlpapi.lib")
#endif

using namespace Navtech;
using namespace cv;
using namespace std;
int downsamplingFactor=1;
uint64_t _lastScanComplete = Helpers::Now();

// DYNAMICALLY CONFIGURABLE PARAMETERS (on the parameter server)
uint16_t threshold_value = 62;
uint16_t dilation_value = 2; //Default Image Slider value
uint16_t gaussian_value = 5; //Default Dilation filter size
uint16_t maxangle = 35;
uint16_t pcl_threshold_value=60;
uint16_t grid_stepSize = 10;
uint16_t grid_stepSize_m;
uint16_t grid_width;
uint16_t grid_height;
uint8_t colormap = 10;
uint16_t level_above_average=19;
uint16_t adaptive_size=2;

bool boundingboxes2 = false;
bool pointcloud = false;
bool LaserScan2 = false;
bool MinAreaRect = false;
bool radarimage=false;
bool radarimageprocessing =false;
bool grid = true;
bool adaptive_threshold=false;
bool longtermaverage = false;
bool ThreeDScanApp;
static bool Configuration_NOT_Set = 1;
double grid_opacity = 0.5;

// End of configurable Parameters
uint16_t azimuths = 400;
bool initialisedaverage;
float range_res;
float sin_values [405]={};
float cos_values [405]={};
float Phi = 0;
float last_Phi=0;
pcl::PointCloud<pcl::PointXYZI>::Ptr threeDPCL(new pcl::PointCloud<pcl::PointXYZI>);

image_transport::Publisher CartesianPublisher;
image_transport::Publisher FilteredPublisher;
image_transport::Publisher BoxesPublisher;
ros::Publisher LaserScanPublisher2;
ros::Publisher PointcloudPublisher;
ros::Publisher PointcloudPublisher3d;

ros::Publisher MarkerPub;
cv::Mat Long_Term_Average;


void InitialiseParameters(float &range_res,uint16_t &azimuths)
{
    if((ros::param::has("configuration_range_res")) && (ros::param::has("configuration_azimuths"))){
		
		ros::param::getCached("configuration_range_res",range_res);
		int az_tmp;
		ros::param::get("/configuration_azimuths",az_tmp);
		azimuths = az_tmp;
		std::cout<<"\nRange Resolution Set from Parameter Server: "<<range_res<<"m\n";
		std::cout<<"Number of Azimuths Set from Parameter Server: "<<az_tmp<<"\n"<<std::endl;

		for (int i=0;i<azimuths;i++){
			float theta=(float)i*2*M_PI/azimuths;
			sin_values[i]=sin(theta);
			cos_values[i]=cos(theta);
		}
		for (int i=az_tmp;i<azimuths+5;i++){
			float theta=(float)(i-azimuths)*2*M_PI/azimuths;
			sin_values[i]=sin(theta);
			cos_values[i]=cos(theta);
		}
	}
	else{
		std::cout<<"\nConfiguration Settings not loaded from ros::param server..\nIf doing data playback, set parameters in server manually using ROS param CLI.\nParameters required are: configuration_range_res and configuration_azimuths\nExiting...";
		}
}

void PublishPointcloud(cv_bridge::CvImagePtr &cv_polar_image, uint16_t &pcl_threshold_value, uint16_t &maxangle, float &range_res)
{
	float theta;
	cv::Mat navigation_image = (cv_polar_image->image - (double)pcl_threshold_value) * 255.0 / (255.0 - pcl_threshold_value);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

	for (int bearing = 0; bearing < cv_polar_image->image.rows; bearing++)
	{
		theta = ((float)(bearing) / cv_polar_image->image.rows) * 2 * M_PI;
		for (size_t ind =1; ind < cv_polar_image->image.cols; ind++)
		{
			pcl::PointXYZI p;
			if ((theta < maxangle * M_PI / 180) || (theta > 2 * M_PI - maxangle * M_PI / 180))
			{
				//The coordinate of the point is taken from the depth map
				//Y and Z  taken negative to immediately visualize the cloud in the right way
				p.x = range_res * ind * cos(theta);
				p.y = range_res * ind * sin(theta);
				//Coloring the point with the corrispondent point in the rectified image
				p.intensity = navigation_image.at<uchar>(bearing, ind);
				p.z = 0;//((float)p.intensity / 64);//0; // Uncomment to make height of pixel proportional to Radar cross sectional area ((float)p.intensity / 32);
			}
			//Insert point in the cloud, cutting the points that are too distant
			if (cv_polar_image->image.at<uchar>(bearing, ind) > pcl_threshold_value)
				cloud->points.push_back(p);
		}
	}
	cloud->width = (int)cloud->points.size();
	cloud->height = 1;
	cloud->header.frame_id = "navtech";
	pcl_conversions::toPCL(cv_polar_image->header.stamp,cloud->header.stamp);//pcl_conversions::toPCL(cv_polar_image->header.stamp,cloud->header.stamp);
	PointcloudPublisher.publish(cloud);
}

void PublishLaserScan2(cv::Mat &new_im2,float range_resolution, uint16_t range_bins, uint16_t ScanAzimuths, int radarbins,cv_bridge::CvImagePtr &cv_polar_image)
{
	sensor_msgs::LaserScan scan2;
	scan2.angle_increment = 2 * M_PI / (float)ScanAzimuths;
	scan2.time_increment = 1 / ((float)4 * (float)ScanAzimuths);
	scan2.scan_time = 1 / (float)4;
	scan2.range_min = 0.0;
	scan2.range_max = 500.0;						//Set MaxRange of Scan
	scan2.ranges.resize(ScanAzimuths);
	std::cout<<"\nrange_bins, radar_bins: "<<range_bins<<", "<<radarbins;
	for (int i = 0; i <ScanAzimuths; i++) //all azimuths
	{
		for (int j = 30; j < range_bins; j++) //bin ranges
		{
			if (j == range_bins - 2)
			{
				scan2.ranges[i] = (float)j * range_resolution*(float)range_bins/(float)radarbins;	
				break;
			}

			if (new_im2.at<uchar>(i, j) == 0) //if no edge is at this location, carry on searching along azimuth
			{
			}
			else //else, an edge is there - assign edge value
			{
				scan2.ranges[i] = (float)j * range_resolution*(float)range_bins / (float)radarbins;
				break;
			}
		}
	}
	scan2.header.stamp=cv_polar_image->header.stamp;//cv_polar_image->header.stamp;
	scan2.header.frame_id = "navtech";
	LaserScanPublisher2.publish(scan2);
}

void PublishProcessedImage(cv_bridge::CvImagePtr &Input_cv_image){
    	cv::Mat radar_image_polar_copy ;
		Input_cv_image->image.copyTo(radar_image_polar_copy);
 		cv::Mat maskmat(Input_cv_image->image.size(),CV_8UC1);
        uint16_t cartesian_rows = Input_cv_image->image.cols;
		uint16_t cartesian_cols = Input_cv_image->image.cols;
		cv::Mat radar_image_cart = cv::Mat::zeros(cartesian_rows, cartesian_cols, CV_8UC1);
		cv::Point2f center((float)cartesian_cols / 2, (float)cartesian_rows / 2);
		double maxRadius = min(center.y, center.x);
		int flags = cv::INTER_LINEAR + cv::WARP_FILL_OUTLIERS;
		if(radarimageprocessing){
			cv::GaussianBlur(radar_image_polar_copy, radar_image_polar_copy, cvSize(gaussian_value * 2 + 1, gaussian_value * 2 + 1), 0);
            cv::Mat processed_im;
            cv::threshold(radar_image_polar_copy, radar_image_polar_copy, threshold_value, 255, CV_THRESH_TOZERO);
            cv::Canny(radar_image_polar_copy, radar_image_polar_copy, 50, 100);
            cv::dilate(radar_image_polar_copy, radar_image_polar_copy, cv::getStructuringElement(cv::MORPH_RECT, cvSize(dilation_value + 1, dilation_value + 1)));
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(radar_image_polar_copy, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE); //Save only outer contours
            cv::drawContours(maskmat, contours, -1, 255, -1);
            cv::warpPolar(maskmat, radar_image_cart, radar_image_cart.size(), center, maxRadius, flags + cv::WARP_INVERSE_MAP);
		}
		else{
            
			cv::warpPolar(radar_image_polar_copy, radar_image_cart, radar_image_cart.size(), center, maxRadius, flags + cv::WARP_INVERSE_MAP);			
		}
        cv::rotate(radar_image_cart, radar_image_cart, cv::ROTATE_90_COUNTERCLOCKWISE);
        cv::bitwise_not(radar_image_cart,radar_image_cart);
		cv::Mat radar_image_color;
		cv::applyColorMap(radar_image_cart, radar_image_color, colormap);
        if (grid){
            cv::Mat radar_grid;
            grid_width = radar_image_color.size().width;
            grid_height = radar_image_color.size().height;
            radar_image_color.copyTo(radar_grid);
            int start_point_y = (grid_height / 2) % grid_stepSize_m;
            int start_point_x = (grid_width / 2) % grid_stepSize_m;
            for (int i = start_point_y; i < grid_height; i += grid_stepSize_m)
                cv::line(radar_grid, Point(0, i), Point(grid_width, i), cv::Scalar(255, 255, 255));

            for (int i = start_point_x; i < grid_width; i += grid_stepSize_m)
                cv::line(radar_grid, Point(i, 0), Point(i, grid_height), cv::Scalar(255, 255, 255));
            addWeighted(radar_image_color, grid_opacity, radar_grid, 1 - grid_opacity, 0.0, radar_image_color);
        }
		sensor_msgs::ImagePtr CartesianMsg = cv_bridge::CvImage(Input_cv_image->header, "rgb8", radar_image_color).toImageMsg();
		CartesianPublisher.publish(CartesianMsg);
}

void FindEdges2(cv_bridge::CvImagePtr &inputImage,uint16_t &gaussian_value, cv::Mat &OutputMatImage, uint16_t &threshold_value, uint16_t &dilation_value,uint16_t maxangle)
{
    cv::Mat radar_image_rcs;
	inputImage->image.copyTo(radar_image_rcs);
	if(adaptive_threshold){
		cv::adaptiveThreshold(OutputMatImage,OutputMatImage,255,ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY,adaptive_size*2+1,-(double)level_above_average);
	}
	cv::GaussianBlur(inputImage->image, inputImage->image, cvSize(gaussian_value * 2 + 1, gaussian_value * 2 + 1), 0);
	if(AZIMUTH_AVERAGING){
		cv::Mat mean;
		cv::reduce(inputImage->image,mean,0,CV_REDUCE_AVG,CV_8UC1);
		inputImage->image.copyTo(OutputMatImage);
		for(int i=0;i<inputImage->image.rows;i++)
		{
			cv::Mat temp_im;
			cv::threshold(inputImage->image.row(i), inputImage->image.row(i), (int)mean.at<uchar>(0,i)+threshold_value, 255, CV_THRESH_TOZERO);
			inputImage->image.row(i).copyTo(OutputMatImage.row(i));
		}
	}
	else
	{
	cv::threshold(inputImage->image, OutputMatImage, threshold_value, 255, CV_THRESH_TOZERO);
	}
	cv::vconcat(OutputMatImage,OutputMatImage.rowRange(0,5),OutputMatImage);
	
	uint16_t deltaRow = inputImage->image.rows/2 - std::round(((float)maxangle/360.0)*(inputImage->image.rows));
	OutputMatImage.rowRange(inputImage->image.rows/2 - deltaRow,inputImage->image.rows/2 +deltaRow).setTo(0);
	cv::Canny(OutputMatImage, OutputMatImage, 50, 100);
}

void PublishBoundingBoxes(cv_bridge::CvImagePtr &cv_polar_image,cv::Mat &EdgesMat,float &range_resolution){
    cv::dilate(EdgesMat, EdgesMat, cv::getStructuringElement(cv::MORPH_RECT, cvSize(dilation_value + 1, dilation_value + 1)));
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(EdgesMat, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE); //Save only outer contours
    visualization_msgs::MarkerArray Marker_Array;
    Marker_Array.markers.resize(contours.size());
    cv::Mat radar_image_rcs;
	cv_polar_image->image.copyTo(radar_image_rcs);
    //Draw Box Contours Round Features
    if (MinAreaRect)
        {
            geometry_msgs::Point p;
            p.z=0;
            for (unsigned int i = 0; i < contours.size(); i++)
            {
                
                Marker_Array.markers[i].header.frame_id = "navtech";
                Marker_Array.markers[i].header.stamp = cv_polar_image->header.stamp;//cv_polar_image->header.stamp;
                Marker_Array.markers[i].action = visualization_msgs::Marker::ADD;
                Marker_Array.markers[i].type = visualization_msgs::Marker::LINE_STRIP;
                Marker_Array.markers[i].pose.orientation.w = 1.0;
                Marker_Array.markers[i].id = i;
                Marker_Array.markers[i].scale.x = .3;
                Marker_Array.markers[i].color.b = 0;//1.0;
                Marker_Array.markers[i].color.a = 1.0;
                Marker_Array.markers[i].lifetime=ros::Duration(.25);
                
                
                Mat maskmat(radar_image_rcs.size(),CV_8UC1);
                double minVal;
                double maxVal;
                Scalar mean;
                cv::Point p1,p2;

                drawContours(maskmat, contours, i, 255, -1);
                cv::minMaxLoc(radar_image_rcs, &minVal, &maxVal, &p1,&p2, maskmat);
                mean = cv::mean(radar_image_rcs, maskmat);
                if(mean[0]>127){mean[0]=127;}
                Marker_Array.markers[i].color.r = (127+mean[0])/255;
                Marker_Array.markers[i].color.g = (127-mean[0])/255;

                cv::RotatedRect rect = cv::minAreaRect(contours[i]);
                cv::Point2f points[4];
                rect.points(points);
                p.z=1;
                for (unsigned int j = 0; j<4; j++)
                {
                    p.y=points[j].x*sin(points[j].y*2*M_PI/(float)azimuths)*range_resolution;
                    p.x=points[j].x*cos(points[j].y*2*M_PI/(float)azimuths)*range_resolution;
                    Marker_Array.markers[i].points.push_back(p);
                }
                    p.y=points[0].x*sin(points[0].y*2*M_PI/(float)azimuths)*range_resolution;
                    p.x=points[0].x*cos(points[0].y*2*M_PI/(float)azimuths)*range_resolution;
                    Marker_Array.markers[i].points.push_back(p);
                    Marker_Array.markers.push_back(Marker_Array.markers[i]);	
            }	
        }	
        
    else{
        for (unsigned int i = 0; i < contours.size(); i++)
        {
            Marker_Array.markers[i].header.frame_id = "navtech";
            Marker_Array.markers[i].header.stamp = cv_polar_image->header.stamp;
            Marker_Array.markers[i].action = visualization_msgs::Marker::ADD;
            Marker_Array.markers[i].pose.orientation.w = 1.0;
            Marker_Array.markers[i].id = i;
            Marker_Array.markers[i].type = visualization_msgs::Marker::LINE_LIST;
            Marker_Array.markers[i].scale.x = .1;
            Marker_Array.markers[i].scale.z = .1;
            Marker_Array.markers[i].scale.y = .1;
            Marker_Array.markers[i].color.b = 0;//1.0;
            Marker_Array.markers[i].color.a = 1.0;
            Marker_Array.markers[i].lifetime=ros::Duration(.25);	
            std::vector<cv::Point> boundingContour;
            geometry_msgs::Point p;
            p.z=0;

            cv::Mat maskmat(radar_image_rcs.size(),CV_8UC1);
            double minVal;
            double maxVal;
            Scalar mean;
            cv::Point p1,p2;

            drawContours(maskmat, contours, i, 255, -1);
            cv::minMaxLoc(radar_image_rcs, &minVal, &maxVal, &p1,&p2, maskmat);
            mean = cv::mean(radar_image_rcs, maskmat);
            /*if(mean[0]>100){mean[0]=100;}
            if(mean[0]<10){mean[0]=10;}*/
            Marker_Array.markers[i].color.r = maxVal/255;
            Marker_Array.markers[i].color.g = 1.0-maxVal/255;

            for (int k=0;k<10;k++)
            {
                for (unsigned int j = 0; j<contours[i].size(); ++j)
                {
                    p.y=(contours[i][j].x*sin_values[contours[i][j].y])*range_resolution;
                    p.x=(contours[i][j].x*cos_values[contours[i][j].y])*range_resolution;
                    Marker_Array.markers[i].points.push_back(p);
                    if(j+1<contours[i].size())
                        {
                            p.y=(contours[i][j+1].x*sin_values[contours[i][j+1].y])*range_resolution;
                            p.x=(contours[i][j+1].x*cos_values[contours[i][j+1].y])*range_resolution;
                        }
                    else
                        {
                            p.y=(contours[i][0].x*sin_values[contours[i][0].y])*range_resolution;
                            p.x=(contours[i][0].x*cos_values[contours[i][0].y])*range_resolution;
                        
                        }
                    Marker_Array.markers[i].points.push_back(p);
                }
                p.z=p.z+.2;
            }
            Marker_Array.markers.push_back(Marker_Array.markers[i]);
            }
        }
    MarkerPub.publish(Marker_Array);
}

void createThreeDPointCloud(cv::Mat &edgesMatInput,float range_resolution, uint16_t range_bins,uint16_t scanAzimuths,cv_bridge::CvImagePtr &cv_polar_image,uint16_t &maxangle){
	last_Phi=Phi;
	for (int i = 0; i <scanAzimuths; i++) //all azimuths
	{
		float theta = 2*M_PI* i/scanAzimuths; 
		pcl::PointXYZI p;
		for (int j = 30; j < range_bins; j++) //bin ranges
		{
			if (edgesMatInput.at<uchar>(i, j) == 0) //if no edge is at this location, carry on searching along azimuth
			{

			}
			else //else, an edge is there - assign edge value
			{
				if ((theta < maxangle * M_PI / 180) || (theta > 2 * M_PI - maxangle * M_PI / 180)){
					p.x =(float)j * range_resolution* cos(theta);
					p.y = (float)j * range_resolution* sin(theta) * cos(Phi*M_PI/180);
					p.z = (float)j * range_resolution* sin(theta) * sin(Phi*M_PI/180);
					//Coloring the point with the corrispondent point in the rectified image
					if (p.z>18){ p.intensity=255;}
					if (p.z<-2){ p.intensity=0;}
					p.intensity = (int)((p.z+2)*255)/20;
					threeDPCL->points.push_back(p);
				}
				break;
			}
		}
		
	}
}
void ParamCallback(nav_ross::dynamic_paramConfig &config, uint32_t level){
	ThreeDScanApp = config.ThreeDScanApp;
	threshold_value = config.threshold_value;
	dilation_value = config.dilation_value;
	gaussian_value = config.gaussian_value;
	pcl_threshold_value = config.pcl_threshold_value;
	maxangle = config.maxangle;
	colormap = config.colormap;
	boundingboxes2 = config.boundingboxes2;
	MinAreaRect = config.MinAreaRect;
	longtermaverage = config.longtermaverage;
	LaserScan2 = config.LaserScan2_Map;
	pointcloud = config.pointcloud;
	radarimage=config.radarimage;
	radarimageprocessing=config.radarimageprocessing;
	grid = config.grid;
	adaptive_threshold=config.adaptive_threshold;
	level_above_average=config.level_above_average;
	adaptive_size=config.adaptive_size;
	if (grid == true){
		grid_stepSize = config.grid_stepSize;
		grid_stepSize_m = grid_stepSize / (range_res * 2);
		grid_opacity = config.grid_opacity;
	}
}

void ChatterCallback(const sensor_msgs::ImageConstPtr &radar_image_polar)
{
	_lastScanComplete = Helpers::Now();

	cv_bridge::CvImagePtr cv_polar_image;
	cv_polar_image = cv_bridge::toCvCopy(radar_image_polar, sensor_msgs::image_encodings::MONO8);
	cv_polar_image->header.stamp=radar_image_polar->header.stamp;
	cv::resize(cv_polar_image->image,cv_polar_image->image,cv::Size(),1,1/(float)downsamplingFactor,INTER_NEAREST);
	//range_res=range_res*2;
	uint16_t bins = cv_polar_image->image.rows;
	//uint16_t azimuths = cv_polar_image->image.cols;
	rotate(cv_polar_image->image, cv_polar_image->image, cv::ROTATE_90_COUNTERCLOCKWISE);
	cv::normalize(cv_polar_image->image, cv_polar_image->image, 0, 255, NORM_MINMAX, CV_8UC1);
	cv::Mat edges_Mat;
	float adj_range = range_res*downsamplingFactor;
	if (pointcloud == true){
		PublishPointcloud(cv_polar_image,pcl_threshold_value,maxangle,adj_range);	
	}

    if(radarimage){
		//if(initialisedaverage){
		//	cv_polar_image->image = Long_Term_Average;}
        PublishProcessedImage(cv_polar_image);	
	}
	if(longtermaverage){
		if(initialisedaverage){
			Long_Term_Average = .9*Long_Term_Average + .1*cv_polar_image->image;
			cv_polar_image->image = cv_polar_image->image-Long_Term_Average;
		}
		else{
			Long_Term_Average = cv_polar_image->image;
			initialisedaverage=true;
		}
	}
	else{initialisedaverage=false;}
	
	if(boundingboxes2 || LaserScan2 || ThreeDScanApp){
        FindEdges2(cv_polar_image,gaussian_value,edges_Mat,threshold_value,dilation_value,maxangle);
    }
	
	if (boundingboxes2 == true){
        PublishBoundingBoxes(cv_polar_image,edges_Mat,adj_range);
	}
	
	if (LaserScan2 == true){
		PublishLaserScan2(edges_Mat, adj_range, edges_Mat.cols, edges_Mat.rows, bins,cv_polar_image);
	}

     if(boundingboxes2 || LaserScan2){
        sensor_msgs::ImagePtr FilteredMsg = cv_bridge::CvImage(cv_polar_image->header, "mono8", edges_Mat).toImageMsg();
	    FilteredPublisher.publish(FilteredMsg);
    }
	if(ThreeDScanApp){
		if(Phi != last_Phi){
		createThreeDPointCloud(edges_Mat,adj_range,edges_Mat.cols,edges_Mat.rows,cv_polar_image,maxangle);
		}
		threeDPCL->width = (int)threeDPCL->points.size();
		threeDPCL->height = 1;
		threeDPCL->header.frame_id = "navtech";
		pcl_conversions::toPCL(cv_polar_image->header.stamp,threeDPCL->header.stamp);//pcl_conversions::toPCL(cv_polar_image->header.stamp,cloud->header.stamp);
		PointcloudPublisher3d.publish(threeDPCL);		
	}	
	std:cout<<"\nProcessing time: "<<Helpers::Now()-_lastScanComplete;
	if(Helpers::Now()-_lastScanComplete>150){
		if(downsamplingFactor<8) {
		std::cout<<"Too much to process... Decreasing resolution of image";
		downsamplingFactor++;
		std::cout<<"Downsampling factor is: "<<downsamplingFactor;
		}
		else std::cout<<"Max downsampling reached!";
	}
	if(Helpers::Now()-_lastScanComplete<50){
		if(downsamplingFactor>1) {
		std::cout<<"Available processing power... Using higher resuloution image";
		downsamplingFactor--;
		std::cout<<"Downsampling factor is: "<<downsamplingFactor;
		}
	}
	//if(_lastScanComplete>Helpers::Now())
}
void updatePhi(const std_msgs::Float32::ConstPtr &newPhi){
    
    Phi=2*(newPhi->data)*360/4096;
    std::cout<<Phi<<std::endl;
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "Mapper_V2");
	ROS_INFO("initialized = true");
	ros::NodeHandle n;
	ros::NodeHandle n1;
    InitialiseParameters(range_res,azimuths);
    ros::Subscriber sub2;
	sub2 = n.subscribe("/Navtech/Polar", 10, ChatterCallback);
	ros::Subscriber arduinoSub;
	arduinoSub = n.subscribe("arduino_phi", 1, updatePhi);

	std::cout<<"here"<<std::endl;

	image_transport::ImageTransport it(n1);
	CartesianPublisher = it.advertise("Navtech/Cartesian1", 10);
	FilteredPublisher = it.advertise("Navtech/Filtered", 10);
	LaserScanPublisher2 = n1.advertise<sensor_msgs::LaserScan>("Navtech/scan3", 1);
	PointcloudPublisher = n1.advertise<sensor_msgs::PointCloud2>("Navtech/FilteredPointcloud", 1);
	PointcloudPublisher3d = n1.advertise<sensor_msgs::PointCloud2>("Navtech/3dPCL", 1);

	MarkerPub = n1.advertise<visualization_msgs::MarkerArray>("visualization_markers",1000);

	dynamic_reconfigure::Server<nav_ross::dynamic_paramConfig> srv;
	dynamic_reconfigure::Server<nav_ross::dynamic_paramConfig>::CallbackType f;
	f = boost::bind(&ParamCallback, _1, _2);

	srv.setCallback(f);

	ros::spin(); 
	ROS_INFO("spinning = true");

	return 0;
}