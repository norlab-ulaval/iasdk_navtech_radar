#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <ctime>
#include <opencv2/opencv.hpp>
#include <iostream>

#include "video_capture_manager.h"

int main(int, char**) {

    std::shared_ptr<Video_capture_manager> vid_cap_manager = std::make_shared<Video_capture_manager>();

    auto ret = vid_cap_manager->connect_to_camera("rtsp://admin:Navtech Radar.1@10.77.3.103:554/cam/realmonitor?channel=1&subtype=1&unicast=true&proto=Onvif");

	if (ret){
        vid_cap_manager->test_framerate(4);
	}
	
    vid_cap_manager->disconnect_from_camera();
}
