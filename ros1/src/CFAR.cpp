#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

bool CFAR(cv::Mat &radar_image_polar,int bin,int bearing,int num_train,int num_guard,int alpha)
{
    
    int noiseSum=0;
    int guardSum=0;
   
            for (int i =0;i<num_train;i++)
                {
                    if (bin-i-numgurad<0 || bin+i+num_guard>2856)
                        {std::cout<<"ERROR, Bin range exceeded";
                        return -1;}
                    noiseSum=radar_image_polar.at<uchar>(bin+i+num_guard, bearing)+noiseSum;
                    noiseSum=radar_image_polar.at<uchar>(bin-i-num_guard, bearing)+noiseSum;
                    noiseSum=radar_image_polar.at<uchar>(bin, bearing+i+num_guard)+noiseSum;
                    noiseSum=radar_image_polar.at<uchar>(bin, bearing-i-num_guard)+noiseSum;

                }

            if (radar_image_polar.at<uchar>(bin, bearing)>alpha+(noiseSum/(num_train*4)))
            return true;
            else
            {
                return false;
            }
            

    


}