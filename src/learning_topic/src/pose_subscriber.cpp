/** 
  * This program will subscribe the topic: /turtle1/pose, type of message is turtlesim::Pose
  */

#include <ros/ros.h>
#include "turtlesim/Pose.h"

// This messages callback function will be called after receving the subscribed message
void poseCallback(const turtlesim::Pose::ConstPtr& msg)
{
    // print the receving message
    ROS_INFO("Turtle pose: x:%0.6f, y:%0.6f", msg->x, msg->y);
}

int main(int argc, char **argv)
{
    // initialize the ROS node
    ros::init(argc, argv, "pose_subscriver");
    
    // create node handler
    ros::NodeHandle n;
    
    // create a Subscriber, topic named /turtle/pose, register callback function poseCallback
    ros::Subscriber pose_sub = n.subscribe("/turtle1/pose", 10, poseCallback);
    
    // repeat wait for callback function
    ros::spin();
    
    return 0;
}
