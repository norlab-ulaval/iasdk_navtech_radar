#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>

void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z-3.5));
    tf::Quaternion q;
    q.setW(msg->pose.orientation.w);
    q.setX(msg->pose.orientation.x);
    q.setY(msg->pose.orientation.y);
    q.setZ(msg->pose.orientation.z);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"world","motor"));
    
    static tf::TransformBroadcaster br2;
    tf::Transform transform2;
    transform2.setOrigin(tf::Vector3(2.0,2.0,-2.0));
    tf::Quaternion q2;
    q2.setRPY(0,1.57079633,0);
    transform2.setRotation(q2);
    br2.sendTransform(tf::StampedTransform(transform2,msg->header.stamp,"motor","navtech"));

}

int main(int argc, char** argv){
    ros::init(argc,argv,"radar_tf_broadcaster");
    std::cout<<"TF Node Initialised";
    ros::NodeHandle nodetf;
    ros::Subscriber sub = nodetf.subscribe("base",1000,poseCallback);
    ros::spin();
    return 0;
};