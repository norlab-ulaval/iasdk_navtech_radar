#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <ctime>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <thread>

#include "video_capture_manager.h"

Video_capture_manager::Video_capture_manager(){
}

int Video_capture_manager::connect_to_camera(std::string camera_url)
{
	std::cout << "Connecting to camera..." << std::endl;

	capture = cv::VideoCapture{camera_url};
	if (!capture.isOpened()) {
		std::cout << "Unable to connect to camera" << std::endl;
		return 0;
	}
	else {
		std::cout << "Camera connected" << std::endl;
		std::cout << "Width: " << capture.get(cv::CAP_PROP_FRAME_WIDTH) << std::endl;
		std::cout << "Height: " << capture.get(cv::CAP_PROP_FRAME_HEIGHT) << std::endl;
		std::cout << "Fps: " << capture.get(cv::CAP_PROP_FPS) << std::endl;
		return 1;
	}
	
}

cv::Mat Video_capture_manager::get_latest_frame()
{
	cv::Mat latest_image{ };
	auto start = std::chrono::steady_clock::now();
	auto finish = std::chrono::steady_clock::now();
	std::chrono::duration<double> elapsed = finish - start;
	while (elapsed.count() < 1 / (capture.get(cv::CAP_PROP_FPS) + 1)) {
		start = std::chrono::steady_clock::now();
		capture >> latest_image;
		finish = std::chrono::steady_clock::now();
		elapsed = finish - start;
	}
	return latest_image;
}

void Video_capture_manager::test_framerate(int num_captures)
{
	
	// Flush the capture buffer
	auto start = std::chrono::steady_clock::now();
	auto finish = std::chrono::steady_clock::now();
	std::chrono::duration<double> elapsed = finish - start;
	while (elapsed.count() < 1 / (capture.get(cv::CAP_PROP_FPS) + 1)){
		start = std::chrono::steady_clock::now();
		capture.grab();
		finish = std::chrono::steady_clock::now();
		elapsed = finish - start;
	}
	std::cout << "Buffer cleared" << std::endl;

	std::cout << "Capturing " << num_captures << " frames..." << std::endl;
	
	start = std::chrono::steady_clock::now();

	cv::Mat captured_image{ };
	for (int f = 0; f < num_captures; f++){
		capture >> captured_image;
	}
	
	finish = std::chrono::steady_clock::now();
	elapsed = finish - start;
	std::cout << "Elapsed time: " << elapsed.count() << std::endl;
	auto fps = num_captures / elapsed.count();
	std::cout << "FPS: " << fps << std::endl;
}

void Video_capture_manager::disconnect_from_camera()
{
	std::cout << "Disconnecting from camera..." << std::endl;
	capture.release();
}