/*
Copyright 2016 Navtech Radar Limited
This file is part of iasdk which is released under The MIT License (MIT).
See file LICENSE.txt in project root or go to https://opensource.org/licenses/MIT 
for full license details.
*/


#include <sstream>
#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <math.h>

#include "ros/ros.h"
#include "radarclient.h"
#include "std_msgs/String.h"
#include "nav_ross/nav_msg.h"
//#include "nav_ross/config_msg.h"
#include <opencv2/opencv.hpp>
#include <string>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#ifdef _WIN32
#include <winsock2.h>
#pragma comment(lib, "Ws2_32.lib")
#pragma comment(lib, "Rpcrt4.lib")
#pragma comment(lib, "Iphlpapi.lib")
#endif
using namespace Navtech;

uint16_t _packetCount = 0;

image_transport::Publisher  PolarPublisher;			
uint16_t _lastAzimuth_navigation = 0;
ros::Publisher Radar_Config_Publisher;
uint16_t _lastAzimuth = 0;
uint64_t _lastRotationReset = Helpers::Now();
RadarClientPtr_t _radarClient;
int azimuth_counter = 0;
int azimuth_counter_navigation = 0;
cv::Mat radar_image_polar;
long frame_number = 0;
std::vector<std::vector<std::tuple<float, uint16_t>>> all_peaks_raw;
std::vector<uint16_t> all_azimuths_raw;
std::vector<std::vector<float>> all_peaks_cart;
std_msgs::Header header;  // empty header
std::string check3;
int publish_image = 0;
float range_res;
int range_in_bins;
int azimuths;
int encoder_size;
int bin_size;
int expected_rotation_rate;


void FFTDataHandler(const FFTDataPtr_t& data)
{	

		ros::spinOnce();	
	_packetCount++;
	if (data->Azimuth < _lastAzimuth) {

		auto diff = Helpers::Now() - _lastRotationReset;
		nav_ross::nav_msg msg;
		msg.range_resolution = range_res;
		msg.AzimuthSamples = azimuths;
		msg.EncoderSize = encoder_size;
		msg.BinSize = bin_size;
		msg.RangeInBins = range_in_bins;
		msg.ExpectedRotationRate = expected_rotation_rate;
		//Radar_Config_Publisher.publish(msg);
		if (frame_number > 2) {

			//IF image topic is on
			if (publish_image)															
			{

				header.seq = frame_number;        // user defined counter
				header.stamp.sec =data->NTPSeconds;  // time
				header.stamp.nsec=data->NTPSplitSeconds;

				sensor_msgs::ImagePtr PolarMsg = cv_bridge::CvImage(header, "mono8", radar_image_polar).toImageMsg();
				PolarPublisher.publish(PolarMsg);
				
			}

			
		}
		
		// update/reset variables
		frame_number++;
		azimuth_counter = 0;
		
		_packetCount = 0;
	}
	// populate image
	if (frame_number > 2) {

		int bearing = ((double)data->Azimuth / (double)encoder_size) * (double)azimuths;
		for (size_t i = 0; i < data->Data.size(); i++) {
			radar_image_polar.at<uchar>(i, bearing) = static_cast<int>(data->Data[i]);
			
		}

	}
	_lastAzimuth = data->Azimuth;
}



void ConfigurationDataHandler(const ConfigurationDataPtr_t& data)
{
	Helpers::Log("ConfigurationDataHandler - Expected Rotation Rate [" + std::to_string(data->ExpectedRotationRate) + "Hz]");
	Helpers::Log("ConfigurationDataHandler - Range In Bins [" + std::to_string(data->RangeInBins) + "]");
	Helpers::Log("ConfigurationDataHandler - Bin Size [" + std::to_string(data->BinSize / 10000.0) + "m]");
	Helpers::Log("ConfigurationDataHandler - Range In Metres [" + std::to_string((data->BinSize / 10000.0) * data->RangeInBins) + "m]");
	Helpers::Log("ConfigurationDataHandler - Azimuth Samples [" + std::to_string(data->AzimuthSamples) + "]");
	
	range_res = (data->BinSize / 10000.0);	
	range_in_bins = data->RangeInBins;
	azimuths = data->AzimuthSamples;
	encoder_size = data->EncoderSize;
	bin_size= data->BinSize;
	expected_rotation_rate= data->ExpectedRotationRate;

	float configuration_range_res;
	int configuration_range_in_bins;
	int configuration_azimuths;
	// PUBLISH TO PARAMETER SERVER //
	ros::param::set("/configuration_range_res",range_res);
	ros::param::set("/configuration_azimuths",azimuths);
	ros::param::set("/configuration_range_in_bins",range_in_bins);
	if (ros::param::has("/configuration_range_res") && ros::param::has("/configuration_range_res") && ros::param::has("/configuration_azimuths"))
	{
		std::cout<<"\nRadar Configuration published to parameter server";
	}

	_packetCount = 0;
	_lastAzimuth = 0;
	//_radarClient->SetNavigationGainAndOffset(1.0f, 0.0f);
	//_radarClient->SetNavigationThreshold(70 * 10);

	
	
	ros::NodeHandle nh3("~");

	nh3.getParam("/talker1/param3", check3);
	std::cout << check3 << std::endl;
	ROS_INFO("Got parameter : %s", check3.c_str());
	
	if (check3.compare("image_on") == 0)
	{
		publish_image = 1;
		ROS_INFO("image topic publishing...");
	}
	else
		ROS_INFO("image publisher off");
	


	radar_image_polar = cv::Mat::zeros(range_in_bins, azimuths, CV_8UC1);
	
	_radarClient->StartFFTData();
	//_radarClient->StartNavigationData();
}

void NavigationDataHandler(const NavigationDataPtr_t& data)
{	
	std::cout<<"Got here";
	auto firstRange = std::get<0>(data->Peaks[0]);
	auto firstPower = std::get<1>(data->Peaks[0]);
	auto angle = data->Angle;
	Helpers::Log("NavigationDataHandler - First Target Range [" + std::to_string(firstRange) + "] Power [" + std::to_string(firstPower / 10) + "] Angle " + std::to_string(angle) + "]");
	
}

int32_t main(int32_t argc, char** argv)
{
#ifdef _WIN32
	WSADATA wsaData;
	auto err = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (err != 0) {
		Helpers::Log("Tracker - WSAStartup failed with error [" + std::to_string(err) + "]");
		return EXIT_FAILURE;
	}
#endif
	ros::init(argc, argv, "talker1");
	
	ros::NodeHandle n;
	//Radar_Config_Publisher = n.advertise<nav_ross::nav_msg>("Navtech/Configuration_Topic", 1,true);
	
	ros::NodeHandle node;
	image_transport::ImageTransport it(node);
	PolarPublisher = it.advertise("/Navtech/Polar", 1000);
	
	
	Helpers::Log("Test Client Starting");
	
	_radarClient = std::make_shared<RadarClient>("192.168.0.1");
	_radarClient->SetFFTDataCallback(std::bind(&FFTDataHandler, std::placeholders::_1));
	_radarClient->SetConfigurationDataCallback(std::bind(&ConfigurationDataHandler, std::placeholders::_1));
	_radarClient->Start();
	
	std::this_thread::sleep_for(std::chrono::milliseconds(60*600000));

	Helpers::Log("Test Client Stopping");
	_radarClient->StopNavigationData();
	_radarClient->StopFFTData();

	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		
	_radarClient->SetConfigurationDataCallback();
	_radarClient->SetFFTDataCallback();
	
	_radarClient->Stop();

	Helpers::Log("Test Client Stopped");
	
	return 0;
}
