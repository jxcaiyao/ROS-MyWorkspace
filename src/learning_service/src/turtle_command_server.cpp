/**
 * This program will execute the /turtle_command server, data type is std_srvs/Trigger
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Trigger.h>

ros::Publisher turtle_vel_pub;
bool pubCommand = false;

// callback function of service, input req, output res
bool commandCallback(std_srvs::Trigger::Request &req, 
                     std_srvs::Trigger::Response &res)
{
    pubCommand = !pubCommand;

    // print request data
    ROS_INFO("Publish turtle velocity command [%s]", pubCommand==true?"Yes":"No");

    // configure feedback data
    res.success = true;
    res.message = "Change turtle command state!";

    return true;
}

int main(int argc, char **argv)
{
    // initialize ROS node
    ros::init(argc, argv, "turtle_command_server");

    // create a node handler
    ros::NodeHandle n;

    // create a server named /turtle_command, register a function commandCallback
    ros::ServiceServer command_service = n.advertiseService("/turtle_command", commandCallback);

    // create a pulisher, publish a topic named /turtle1/cmd_vel, type of messages is geometry_msgs::Twist, length of queue is 10
    turtle_vel_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);

    // repeat waiting for callback
    ROS_INFO("Ready to receive turtle command.");

    // set frequency of repeat
    ros::Rate loop_rate(10);

    while(ros::ok()){
        // check the queue of callback function
        ros::spinOnce();

        // if the variable is true, publish the velocity command
        if(pubCommand){
            geometry_msgs::Twist vel_msg;
            vel_msg.linear.x = 0.5;
            vel_msg.angular.z = 0.2;
            turtle_vel_pub.publish(vel_msg);
        }

        // delay according to the frequency of repeat
        loop_rate.sleep();
    }
}