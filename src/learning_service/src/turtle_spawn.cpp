/**
 * This example will request the service /spawn, data type is turtlesim::Spawn
 */

#include <ros/ros.h>
#include <turtlesim/Spawn.h>

int main(int argc, char **argv)
{
    // initialize ROS node
    ros::init(argc, argv, "turtle_spawn");
    
    // create node handler
    ros::NodeHandle node;
    
    // create a service client after finding /spawn service, linking the service named /spawn
    ros::service::waitForService("/spawn");
    ros::ServiceClient add_turtle = node.serviceClient<turtlesim::Spawn>("/spawn");
    
    // initialize the request data of turtlesim::Spawn
    turtlesim::Spawn srv;
    srv.request.x = 2.0;
    srv.request.y = 2.0;
    srv.request.name = "turtle2";
    
    // call request service
    ROS_INFO("Call service to spawn turtle[x:%0.6f, y:%0.6f, name:%s]",
              srv.request.x, srv.request.y, srv.request.name.c_str());
              
    add_turtle.call(srv);
    
    // print the result of service call
    ROS_INFO("Spawn turtle successfully [name:%s]", srv.response.name.c_str());
    
    return 0;
}
