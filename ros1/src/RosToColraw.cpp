// Import dependencies //

#include "radarclient.h"

#include <iostream>
#include <string>
#include <stdexcept>
#include <fstream>
#include <assert.h>
#include <ctime>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <dynamic_reconfigure/server.h>
#include <dynamic_paramConfig.h>
#include <time.h>
#include <ros/duration.h>
#include <ros/time.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>


#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#ifdef _WIN32
#include <winsock2.h>
#pragma comment(lib, "Ws2_32.lib")
#pragma comment(lib, "Rpcrt4.lib")
#pragma comment(lib, "Iphlpapi.lib")
#endif
#ifdef _MSC_VER
#else
#include <byteswap.h>
#define _byteswap_uint64(x) bswap_64(x)
#define _byteswap_ulong(x) bswap_32(x);
#define _byteswap_ushort(x) bswap_16(x);

#endif
bool global_useRosTime = false;
ros::Subscriber rostocolraw;
uint8_t Network_Version = 1;
float range_res;
uint16_t range_in_bins;
uint16_t azimuths; 
uint16_t encoder_size;
uint16_t rotation_speed;
uint16_t packet_rate;
float range_gain;
float range_offset;
ros::Time begin;
ros::Duration durationOfSplitFile(60000.0f);
uint64_t previousticks=0;
ros::Time previoustime;
uint32_t Length;
uint8_t Type;
uint32_t Top;
uint32_t Bottom;
uint16_t Sweep_Counter=0;
std::string Filename;
static bool firstpass = 1;
unsigned char footprint[] = {0x00, 0x01, 0x03, 0x03, 0x07, 0x07, 0x0f, 0x0f, 0x1f, 0x1f, 0x3f, 0x3f, 0x7f, 0x7f, 0xfe, 0xfe};
namespace FloatToNetwork
{
    union v
        {
            float f;
            uint32_t i;
        };
}
void Set_FileName(std::string &Filename){
    std::time_t t = std::time(nullptr);
    char mbstr[15];
    std::strftime(mbstr, sizeof(mbstr), "%Y%m%d%H%M%S", std::localtime(&t));
    std::string output;
    for (int i = 0; i < sizeof(mbstr); i++) {
        output = output + mbstr[i];
    }
    output.erase(sizeof(mbstr)-1,1);
    std::string beginning = "Ride1_1_";
    std::string end = "_group1.colraw";
    std::string total = beginning + output + end;
    Filename=total;    
    std::cout<<total;
}


//Function: Write Config To File - Done

void WriteConfigToFile(float &range_res, uint16_t &range_in_bins, uint16_t &azimuths,uint16_t &encoder_size, uint16_t &rotation_speed,uint16_t &packet_rate,float &range_gain,float &range_offset)
{
    char buffer[13+22+20];
    Length = 42;
    Type = 10;
    Top = 0;
    Bottom = 25903964;
    uint16_t Azimuth_Samples = azimuths;
    uint16_t Bin_Size = range_res * 10000;
    uint16_t Range_In_Bins = range_in_bins;
    uint16_t Encoder_Size = encoder_size;
    uint16_t Rotation_Speed = rotation_speed;
    uint16_t Packet_Rate = packet_rate;
    uint32_t Payload_Size = 20;
        
    uint32_t Network_Length = _byteswap_ulong(Length);
    uint8_t Network_Type = Type;
    uint32_t Network_Top = htonl(Top);
    //uint32_t Network_Bottom = htonl(Bottom);
    uint32_t Network_Bottom = Bottom;    
    uint32_t Network_Payload_Size = htonl(Payload_Size);
    uint16_t Network_Azimuth_Samples = htons(Azimuth_Samples);
    uint16_t Network_Bin_Size = htons(Bin_Size);
    uint16_t Network_Range_In_Bins = htons(Range_In_Bins);
    uint16_t Network_Encoder_Size = htons(Encoder_Size);
    uint16_t Network_Rotation_Speed = htons(Rotation_Speed);
    uint16_t Network_Packet_Rate = htons(Packet_Rate);

    FloatToNetworking::v value;
    value.f=range_gain;
    uint32_t Network_Range_Gain = htonl(value.i);
    
    FloatToNetworking::v value2;
    value2.f=range_offset;
    uint32_t Network_Range_Offset = htonl(value2.i);

    int index = 0;
    //Write Length
        memcpy(&buffer[0+index],&Length,sizeof(Network_Length));
        index+=sizeof(Network_Length);
    //Write Type
        memcpy(&buffer[0+index],&Network_Type,sizeof(Network_Type));
        index+=sizeof(Network_Type);
    //Write Top
        memcpy(&buffer[0+index],&Network_Top,sizeof(Network_Top));
        index+=sizeof(Network_Top);
    //Write Bottom
        memcpy(&buffer[0+index],&Network_Bottom,sizeof(Network_Bottom));
        index+=sizeof(Network_Bottom);
    //Write Footprint
        memcpy(&buffer[0+index],&footprint,sizeof(footprint));
        index+=sizeof(footprint);
    //Write Version
        memcpy(&buffer[0+index],&Network_Version,sizeof(Network_Version));
        index+=sizeof(Network_Version);
    //Write Msg ID
        memcpy(&buffer[0+index],&Network_Type,sizeof(Network_Type));
        index+=sizeof(Network_Type);
    //Write Payload Size
        memcpy(&buffer[0+index],&Network_Payload_Size,sizeof(Network_Payload_Size));
        index+=sizeof(Network_Payload_Size);
    //Write Azimuth Samples
        memcpy(&buffer[0+index],&Network_Azimuth_Samples,sizeof(Network_Azimuth_Samples));
        index+=sizeof(Network_Azimuth_Samples);
    //Write Bin Size
        memcpy(&buffer[0+index],&Network_Bin_Size,sizeof(Network_Bin_Size));
        index+=sizeof(Network_Bin_Size);
    //Write Range In Bins
        memcpy(&buffer[0+index],&Network_Range_In_Bins,sizeof(Network_Range_In_Bins));
        index+=sizeof(Network_Range_In_Bins);
    //Write Encoder Offset
        memcpy(&buffer[0+index],&Network_Encoder_Size,sizeof(Network_Encoder_Size));
        index+=sizeof(Network_Encoder_Size);
    //Write Rotation Speed
        memcpy(&buffer[0+index],&Network_Rotation_Speed,sizeof(Network_Rotation_Speed));
        index+=sizeof(Network_Rotation_Speed);
    //Write Packet Rate
        memcpy(&buffer[0+index],&Network_Packet_Rate,sizeof(Network_Packet_Rate));
        index+=sizeof(Network_Packet_Rate);
    //Write Range Gain
        memcpy(&buffer[0+index],&Network_Range_Gain,sizeof(Network_Range_Gain));
        index+=sizeof(Network_Range_Gain);
    //Write Range Offset
        memcpy(&buffer[0+index],&Network_Range_Offset,sizeof(Network_Range_Offset));
        index+=sizeof(Network_Range_Offset);
    //Write To Buffer
        std::ofstream file;
        std::cout<<"\nOpening file... " + Filename;
        file.open(Filename,std::ios_base::binary);
        if (file.is_open()){
            std::cout<<"\nFILE OPENED";
            file.write(*&buffer, sizeof(buffer));
            file.close();
        }
        else{
            std::cout<<"Could Not Open File - Check to see if File already exists in location";
        }

}

// Function: Get and Set config params - Done
void SetParams(float &range_res, uint16_t &range_in_bins, uint16_t &azimuths,uint16_t &encoder_size, uint16_t &rotation_speed,uint16_t &packet_rate,float &range_gain,float &range_offset)
{ 

    float local_range_res;
    int local_range_in_bins;
    int local_azimuths;
    //If they exist in paramserver
   
    if(ros::param::get("/configuration_range_res",local_range_res))
    {
        range_res=local_range_res;
    }
    else
    {
        std::cout<<"\nNo Range Resolution In Parameter Server...\nEnter value here (in meters)e.g. 0.175\n";
        std::cin>>range_res;
        if(!std::cin){std::cout<<"Invalid Params. Try Again";
        std::cin.clear();
        std::cin.ignore(10000,'\n');
        SetParams(range_res,range_in_bins,azimuths,encoder_size,rotation_speed,packet_rate,range_gain,range_offset);
        }
    }
    if(ros::param::get("/configuration_azimuths",local_azimuths))
    {
        azimuths=local_azimuths;
    }
    else
    {
        std::cout<<"\nNo Number of Azimuths In Parameter Server...\nEnter value here e.g. 400\n";
        std::cin>>azimuths;
        if(!std::cin)
        {
        std::cout<<"Invalid Params. Try Again";
        std::cin.clear();
        std::cin.ignore(10000,'\n');
        SetParams(range_res,range_in_bins,azimuths,encoder_size,rotation_speed,packet_rate,range_gain,range_offset);
        }
    }
    if(ros::param::get("/configuration_range_in_bins",local_range_in_bins))
    {
        range_in_bins=local_range_in_bins;
    }
    else
    {
        std::cout<<"\nNo Range In Bins In Parameter Server...\nEnter value here (in bins)e.g. 2856\n";
        std::cin>>range_in_bins;
            if(!std::cin){std::cout<<"Invalid Params. Try Again";
            std::cin.clear();
            std::cin.ignore(10000,'\n');
            SetParams(range_res,range_in_bins,azimuths,encoder_size,rotation_speed,packet_rate,range_gain,range_offset);
        }
    }
    std::cout<<"\nEncoder Size...\nEnter value here e.g. 5600\n";
    std::cin>>encoder_size;
    if(!std::cin)
    {
        std::cout<<"Invalid Params. Try Again";
        std::cin.clear();
        std::cin.ignore(10000,'\n');
        SetParams(range_res,range_in_bins,azimuths,encoder_size,rotation_speed,packet_rate,range_gain,range_offset);}

    std::cout<<"\nRotation Speed...\nEnter value here (in mHz)e.g. 4000\n";
    std::cin>>rotation_speed;
    if(!std::cin){
        std::cout<<"Invalid Params. Try Again";
        std::cin.clear();
        std::cin.ignore(10000,'\n');
        SetParams(range_res,range_in_bins,azimuths,encoder_size,rotation_speed,packet_rate,range_gain,range_offset);}

    std::cout<<"\nPacket Rate...\nEnter value here (Rot Speed x Azimuths)e.g. 1600\n";
    std::cin>>packet_rate;
    if(!std::cin){std::cout<<"Invalid Params. Try Again";
        std::cin.clear();
        std::cin.ignore(10000,'\n');
        SetParams(range_res,range_in_bins,azimuths,encoder_size,rotation_speed,packet_rate,range_gain,range_offset);}
        
    range_gain=0.99964375;
    range_offset=-0.311984064;
}


// Callback Function: 
    // Get Data + Timestamp //
    // Write To File In Correct Structure //
void DataHandler(const sensor_msgs::ImageConstPtr &radar_image_polar)
{
    uint64_t ticks;
    if (firstpass)
        {
        firstpass=0;
        begin = ros::Time::now();
        previousticks=36530730-(uint64_t)((uint64_t)1 * 10 * 100000000)/(azimuths*rotation_speed);
        ticks=36530730;
        
        cv_bridge::CvImagePtr cv_polar_image;
        cv_polar_image = cv_bridge::toCvCopy(radar_image_polar, sensor_msgs::image_encodings::MONO8);
        if(cv_polar_image->header.stamp.sec ==0 || global_useRosTime)
        {
            previoustime=begin;
        }
        else{
            begin=radar_image_polar->header.stamp;
            previoustime=radar_image_polar->header.stamp;
        }
        std::cout<<"\nHeader Time = "<<previoustime;

    }
    else{
        cv_bridge::CvImagePtr cv_polar_image;
        cv_polar_image = cv_bridge::toCvCopy(radar_image_polar, sensor_msgs::image_encodings::MONO8);

        //Set header time of frame
        if(cv_polar_image->header.stamp.sec ==0 || global_useRosTime)
        {
            cv_polar_image->header.stamp=ros::Time::now();//previoustime + ros::Duration(.25);
            
        }
        else{
            cv_polar_image->header.stamp=radar_image_polar->header.stamp;
        }
        std::cout<<"\nrows, cols = "<<cv_polar_image->image.rows<<", "<<cv_polar_image->image.cols;
        std::cout<<"\nHeader Time = "<<cv_polar_image->header.stamp;
        std::cout<<"\nduration so far, duration max: "<<(cv_polar_image->header.stamp-begin).toSec()<<" vs "<<durationOfSplitFile.toSec();
        if((cv_polar_image->header.stamp-begin).toSec()>durationOfSplitFile.toSec()){
            begin = cv_polar_image->header.stamp;
            Set_FileName(Filename);
            WriteConfigToFile(range_res,range_in_bins,azimuths,encoder_size,rotation_speed,packet_rate,range_gain,range_offset);

        }

        //Get Ticks
        ticks = previousticks+(uint64_t)((uint64_t)1 * 10 * 1000000000)/(azimuths*rotation_speed);
        uint64_t currenttick = ticks;
        for(int az=0; az<azimuths;az++)
        {
            uint16_t azimuth_for_writing = az*14;
            char *buffer = new char[13+22+14+range_in_bins];
            char* Topbytes = new char[4];
            char* Bottombytes = new char[4];
            int index = 0;
            uint32_t Payload_Size = range_in_bins +14;
            ticks = currenttick + (uint64_t)((uint64_t)az * 10 * 1000000000)/(azimuths*rotation_speed);
            double DeltaAzimuthTime =(azimuths-1-az)*((cv_polar_image->header.stamp-previoustime).toSec())/azimuths; 
            ros::Time DeltaAzimuthRosTime(DeltaAzimuthTime);
            ros::Duration currenttime = cv_polar_image->header.stamp - DeltaAzimuthRosTime;
            Length=14+range_in_bins+22;
            Type=30;
            uint32_t Network_Length = _byteswap_ulong(Length);
            uint8_t Network_Type = Type;
            uint16_t Network_Azimuth = _byteswap_ushort(azimuth_for_writing);
            uint64_t Network_ticks = _byteswap_uint64(ticks);
            uint32_t Network_Payload_Size = htonl(Payload_Size);
            uint64_t NetTicks = _byteswap_uint64(ticks);
            uint32_t lowbits = NetTicks >> 32;
            uint32_t highbits = NetTicks & 0xFFFFFFFF00000000;
            lowbits = htonl(lowbits);
            highbits = htonl(highbits);
            memcpy(&Topbytes[0],&highbits,4);
            memcpy(&Bottombytes[0],&lowbits,4);
            //Write File Record Header
            //Write Length
                memcpy(&buffer[0+index],&Length,sizeof(Network_Length));
                index+=sizeof(Network_Length);
            //Write Type
                memcpy(&buffer[0+index],&Network_Type,sizeof(Network_Type));
                index+=sizeof(Network_Type);
            //Write Top
                memcpy(&buffer[0+index],Topbytes,4);
                index+=4;
            //Write Bottom
                memcpy(&buffer[0+index],Bottombytes,4);
                index+=4;
            //Write Footprint
                memcpy(&buffer[0+index],&footprint,sizeof(footprint));
                index+=sizeof(footprint);
            //Write Version
                memcpy(&buffer[0+index],&Network_Version,sizeof(Network_Version));
                index+=sizeof(Network_Version);
            //Write Msg ID
                memcpy(&buffer[0+index],&Network_Type,sizeof(Network_Type));
                index+=sizeof(Network_Type);
            //Write Payload Size
                memcpy(&buffer[0+index],&Network_Payload_Size,sizeof(Network_Payload_Size));
                index+=sizeof(Network_Payload_Size);
            //Get Offset, Write Offset to buffer
                uint16_t FFT_Data_Offset = 14;
                uint16_t Network_FFT_Data_Offset = htons(FFT_Data_Offset);
                memcpy(&buffer[0+index],&Network_FFT_Data_Offset,sizeof(Network_FFT_Data_Offset));
                index+=sizeof(Network_FFT_Data_Offset);
            //Get Sweep Counter, increment per packet,rollover at max type, Write to buffer
                uint16_t Network_Sweep_Counter = htons(Sweep_Counter);
                memcpy(&buffer[0+index],&Network_Sweep_Counter,sizeof(Network_Sweep_Counter));
                index+=sizeof(Network_Sweep_Counter);
            //Get Azimuth, Write Azimuth to buffer
                memcpy(&buffer[0+index],&Network_Azimuth,sizeof(Network_Azimuth));
                    index+=sizeof(Network_Azimuth);
            //Get Time, Write Seconds to buffer
                uint32_t Seconds = currenttime.sec;
                uint32_t Network_Seconds = Seconds;//htonl(Seconds);
                memcpy(&buffer[0+index],&Network_Seconds,sizeof(Network_Seconds));
                index+=sizeof(Network_Seconds);
            //Write Split Seconds to buffer
                uint32_t SplitSeconds = currenttime.nsec;
                uint32_t Network_Split_Seconds = SplitSeconds;//htonl(SplitSeconds);
                memcpy(&buffer[0+index],&Network_Split_Seconds,sizeof(Network_Split_Seconds));
                index+=sizeof(Network_Split_Seconds);
            //Get FFT Data, Write FFT Data to buffer
                for (int bin = 0; bin < range_in_bins; bin++)
                    {
                        //fill buffer with power values
                        buffer[index]=cv_polar_image->image.at<uchar>(bin,az);
                        index++;

                           // std::cout<<"\nazimuth is:"<<az<<", bin is: "<<bin<<". value is: "<<static_cast<int>(buffer[index-1]);
                    }
            std::ofstream file;
            file.open(Filename,std::ios_base::binary|std::ios_base::app);
        if (file.is_open()){
            file.write(*&buffer, 13+22+14+range_in_bins);
            //std::cout<<"\n size of buffer is: "<<sizeof(buffer);
            file.close();
        }
        else{
            std::cout<<"Could Not Open File - Check to see if File already exists in location";
        }
            delete [] buffer;
            delete [] Topbytes;
            delete [] Bottombytes;
            if (Sweep_Counter<65535)
            {
                Sweep_Counter++;
            }
            else
            {
                Sweep_Counter=0;
            }
        }
        
        previousticks=ticks;
        previoustime=cv_polar_image->header.stamp;
    }
}


// Main //
int main(int argc,char **argv)
{
    std::cout<<"\nInitialising Node RosToColraw...";
    ros::init(argc, argv, "RosToColraw");
    std::cout<<"\nInitialising Node RosToColraw Complete";
    ros::NodeHandle convertingNode;
// Function: Get Config of Radar: If none, user input range res + bins
    std::cout<<"\nSetting Config Parameters. Trying to Get from Parameter Server...";
    SetParams(range_res,range_in_bins,azimuths,encoder_size,rotation_speed,packet_rate,range_gain,range_offset);
// Function Set File Name
    Set_FileName(Filename);
// Function: Write File, Config message
    WriteConfigToFile(range_res,range_in_bins,azimuths,encoder_size,rotation_speed,packet_rate,range_gain,range_offset);
// Subscribe to incoming data + write data
    rostocolraw = convertingNode.subscribe("/Navtech/Polar",1000,DataHandler);
    ros::spin();
    return 0;
}