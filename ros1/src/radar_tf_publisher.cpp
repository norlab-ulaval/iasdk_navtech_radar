#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

int main(int argc,char** argv)
{
 
    ros::init(argc,argv,"tf_publisher_node");
    ros::NodeHandle tf_node;
    ros::Publisher tf_publisher_P;
    tf_publisher_P = tf_node.advertise<geometry_msgs::PoseStamped>("base",10);
    geometry_msgs::PoseStamped P;
        float theta=0.0;
        P.pose.position.x =0;
        P.pose.position.y=0;
        P.pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(0,0,theta);
    while(ros::ok)
    {
        theta=theta+.1;
        P.pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(0,0,theta);
        P.header.frame_id="new";
        P.header.stamp=ros::Time::now();
        tf_publisher_P.publish(P);
        ros::Duration(0.2).sleep();    }
    return 0;
}