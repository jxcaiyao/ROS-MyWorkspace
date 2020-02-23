#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char ** argv)
{
  // initialize a ROS node
  ros::init(argc,argv, "velocity_publisher");

  // create a node handle
  ros::NodeHandle n;

  // create a Publisher, the topic named /turtle1/cmd_vel, message type is geometry_msgs::Twist, and the length of queue is 10
  ros::Publisher turtle_vel_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);

  // set the frequence of repeat
  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok()){
    // initialize the message whose type is geometry_msgs::Twist
    geometry_msgs::Twist vel_msg;
    vel_msg.linear.x = 0.5;
    vel_msg.angular.z = 0.2;

    // publish the message
    turtle_vel_pub.publish(vel_msg);
    ROS_INFO("Publish turtle velocity command[%0.2f m/s, %0.2f rad/s]", vel_msg.linear.x, vel_msg.angular.z);

    // pause according to the frequence of repeat
    loop_rate.sleep();
  }

  return 0;
}
