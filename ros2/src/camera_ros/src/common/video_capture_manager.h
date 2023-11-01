#include <opencv2/opencv.hpp>

class Video_capture_manager {
public:	
    Video_capture_manager();

    int connect_to_camera(std::string camera_url);
    void test_framerate(int num_captures);
    void disconnect_from_camera();
    cv::Mat get_latest_frame();

private:
    cv::VideoCapture capture;
    cv::Mat latest_image{ };
};