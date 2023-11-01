/*
Copyright 2016 Navtech Radar Limited
This file is part of iasdk which is released under The MIT License (MIT).
See file LICENSE.txt in project root or go to https://opensource.org/licenses/MIT
for full license details.
*/


/*
Node Description: Stationary sensor, for detection of moving objects
Publishing Topic: Image topic with objects highlighted
*/



#include "radarclient.h"
#include "nav_ross/nav_msg.h"

#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <cstdint>
#include <functional>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <dynamic_reconfigure/server.h>
#include <dynamic_paramConfig.h>
#include <std_msgs/String.h>


#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
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
#ifdef _WIN32
#include <winsock2.h>
#pragma comment(lib, "Ws2_32.lib")
#pragma comment(lib, "Rpcrt4.lib")
#pragma comment(lib, "Iphlpapi.lib")
#endif
using namespace Navtech;
bool CFAR(const nav_ross::nav_msg::ConstPtr& msg,int bin,int bearing,int num_train,int num_guard,int alpha);
uint16_t _packetCount = 0;
uint16_t _lastAzimuth = 0;
uint64_t _lastRotationReset = Helpers::Now();
RadarClientPtr_t _radarClient;
int azimuth_counter = 0;
int azimuth_counter_navigation = 0;
constexpr int ROWS = 713;				//NUMBER OF PIXELS IN A COLUMN
constexpr int COLS = 713;				//NUMBER OF PIXELS IN A ROW, always set to equal to ROWS
constexpr float PIXEL_SIZE = 4;			//Pixel Size (in Bins). The bigger this is, the faster the update speed, but the more range resolution is lost.
// ROWS Times PIXEL_SIZE must equal or be less than RANGE IN BINS
cv::Mat radar_image_polar; 
cv::Mat radar_image = cv::Mat::zeros(ROWS, COLS, CV_8UC1);
long frame_number = 0;
cv::Mat X;
cv::Mat Y;
cv::Mat cv_polar_image;
cv::Mat Clutter;
std::vector<std::vector<std::tuple<float, uint16_t>>> all_peaks_raw;
std::vector<uint16_t> all_azimuths_raw;
std::vector<std::vector<float>> all_peaks_cart;
std_msgs::Header header;  // empty header
int first_message=1;

ros::Subscriber sub;
std::vector<uint8_t> spikes;
std::vector<uint16_t> distances;
float angle;
float range_res;
const char* trackbar_value = "Tracking";		//Label for Tracking Slider - Any pixels above this slider value will be inside a contour
const char* trackbar_value2 = "Dilation";		//Label for Dilation Size Slider
const char* trackbar_value3 = "Gaussian Size";		//Label for Gaussian Size Slider
const char* trackbar_value4 = "Laplacian";		//Label for Laplacian Slider

int const max_value = 250;						//Max slider value for tracking and image thresholding
int const max_value2 = 20;						//Max slider value for dilation filter size (pixels)
int const max_value3 = 15;						//Max Gauss Size
int const max_value4 = 15;						//Max Laplacian Size	

int threshold_value = 27;						//Default tracking slider value
int threshold_value2 = 2;						//Default Dilation filter size
int threshold_value3 = 2;						//Default Gaussian filter size
int threshold_value4 = 3;						//Default Laplacian filter size



using namespace Navtech;
using namespace cv;
using namespace std;

static void Threshold_Demo(int, void*)
{

}

uint64_t render_time;
uint64_t conversion_time;

void chatterCallback(const sensor_msgs::ImageConstPtr &radar_image_polar)
{
	if (frame_number > 2) {
	cv_bridge::CvImagePtr cv_polar_image;
	cv_polar_image = cv_bridge::toCvCopy(radar_image_polar, sensor_msgs::image_encodings::MONO8);
	cv_polar_image->header.stamp=radar_image_polar->header.stamp;
			rotate(cv_polar_image->image, cv_polar_image->image, cv::ROTATE_90_COUNTERCLOCKWISE);

	uint16_t cartesian_rows = cv_polar_image->image.cols;
	uint16_t cartesian_cols = cv_polar_image->image.cols;
	cv::Mat radar_image_cart = cv::Mat::zeros(cartesian_rows, cartesian_cols, CV_8UC1);
	cv::Point2f center((float)cartesian_cols / 2, (float)cartesian_rows / 2);
	double maxRadius = min(center.y, center.x);
	int flags = cv::INTER_LINEAR + cv::WARP_FILL_OUTLIERS;

			cv::warpPolar(cv_polar_image->image, radar_image_cart, radar_image_cart.size(), center, maxRadius, flags + cv::WARP_INVERSE_MAP);
			cv::flip(radar_image_cart, radar_image_cart, 0);

			cv::Mat image;
			cv::Mat canny_image;
			cv::Mat display_im;
			cv::Mat new_im;
			cv::Mat laplace_im;

			cv::GaussianBlur(radar_image_cart, radar_image_cart, cvSize(threshold_value3 * 2 + 1, threshold_value3 * 2 + 1), 0);
			
			if (frame_number == 3)
			{
				Clutter = radar_image_cart.clone();				
				new_im = radar_image_cart;
			}
			else
			{
				cv::absdiff(radar_image_cart, Clutter, new_im);				
				cv::addWeighted(radar_image_cart, .05, Clutter, .95, 0.0, Clutter);
			}
			
			cv::threshold(new_im, new_im, threshold_value, 255, CV_THRESH_TOZERO);	
					
			cv::cvtColor(Clutter, image, CV_GRAY2BGR);			
			cv::Canny(new_im, new_im, 10, 100);
			cv::imshow("Laplacian_Filter", new_im);
			cv::dilate(new_im, new_im, cv::getStructuringElement(cv::MORPH_RECT, cvSize(threshold_value2 + 1, threshold_value2 + 1)));
			
			std::vector<std::vector<cv::Point > > contours;
			cv::findContours(new_im, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE); //Save only outer contours
			
			//Draw Box Contours Round Features 
			for (unsigned int i = 0; i < contours.size(); ++i)
			{
				cv::RotatedRect rect = cv::minAreaRect(contours[i]);
				cv::Point2f points[4];
				rect.points(points);

				std::vector<cv::Point> boundingContour;
				for (unsigned int j = 0; j < 4; ++j)
				{
					boundingContour.push_back(points[j]);
				}

				std::vector<std::vector<cv::Point>> boxContours;
				boxContours.push_back(boundingContour);

				cv::drawContours(image, boxContours, 0, cvScalar(0, 255, 0), 2);
			}
			cv::imshow("Components", image);
			cv::waitKey(1);
}
frame_number++;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "listener1");
	ROS_INFO("initialized = true");
	ros::NodeHandle n;

	cv::namedWindow("Laplacian_Filter", cv::WINDOW_NORMAL);
	cv::namedWindow("Components", cv::WINDOW_NORMAL);
	cv::createTrackbar(trackbar_value,"Components", &threshold_value, max_value, Threshold_Demo); // Create a Trackbar to choose filter settings

	cv::createTrackbar(trackbar_value2, "Components", &threshold_value2, max_value2, Threshold_Demo); // Create a Trackbar to choose filter settings
	cv::createTrackbar(trackbar_value3, "Laplacian_Filter", &threshold_value3, max_value3, Threshold_Demo); // Create a Trackbar to choose filter settings
	cv::createTrackbar(trackbar_value4, "Laplacian_Filter", &threshold_value4, max_value4, Threshold_Demo); // Create a Trackbar to choose filter settings

	sub = n.subscribe("talker1/Navtech/Polar", 1000, chatterCallback);
	ROS_INFO("subscribed = true");

	ros::spin(); // this is where the magic happens!!
	ROS_INFO("spinning = true");

	return 0;
}
