/**
 * This program will publish the topic: /person_info, type of message is learning_topic::Person
 */

#include <ros/ros.h>
#include "learning_topic/Person.h"

int main(int argc, char **argv)
{
    // initialize ROS node
    ros::init(argc, argv, "person_publisher");
    
    // create node handler
    ros::NodeHandle n;
    
    // create a Publisher, topic named /person_info, type of message is learning_topic::Person, length of queue is 10
    ros::Publisher person_info_pub = n.advertise<learning_topic::Person>("/person_info", 10);
    
    // set the frequency of repeat
    ros::Rate loop_rate(1);
    
    int count = 0;
    while(ros::ok()){
        // initialize a message whose type is learning_topic::Person
        learning_topic::Person person_msg;
        person_msg.name = "Tom";
        person_msg.age  = 18;
        person_msg.sex  = learning_topic::Person::male;
        
        // publish message
        person_info_pub.publish(person_msg);
        
        ROS_INFO("Publish Person Info: name:%s  age:%d  sex:%d",
                  person_msg.name.c_str(), person_msg.age, person_msg.sex);
                  
        // delay according to the repeat frequency 
        loop_rate.sleep();
    }
    
    return 0;
}
