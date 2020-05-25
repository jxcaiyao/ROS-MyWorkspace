#include "ros/ros.h"
#include "ros/console.h"
#include <stdio.h>

#include <numeric>
#include <vector>
#include <Eigen/Eigen>

#include "ros/publisher.h"
#include "ros/subscriber.h"
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include <visualization_msgs/MarkerArray.h>

using namespace std;
using namespace Eigen;

#define particle_num 100
typedef struct particle {
    int id;
    float x;
    float y;
    float theta;
    float weight;
} particle;

class particle_filter{

public:
    particle_filter(ros::NodeHandle &n);
	~particle_filter();
    ros::NodeHandle& n;

    // some params
    float init_x;
    float init_y;
    float init_theta;
    float init_rand_xy;
    float init_rand_theta;

    // subers & pubers
    ros::Subscriber laser_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber map_sub;
    ros::Publisher particles_pub;
    tf::TransformBroadcaster tf_broadcaster;

    // particles 
    particle particles[particle_num];

    // global state
    Eigen::Vector3d state;
    // map
    nav_msgs::OccupancyGrid global_map;
    bool isMapSet = false;

    // set map
    void setMap(nav_msgs::OccupancyGrid input);
    // init the state and particles 
    void init();
    // do Motion from ICP odom
    void doMotion(nav_msgs::Odometry input);
    // do Observation to weight each particle
    void doObservation(sensor_msgs::LaserScan input);
    // publish the particles and the estimated state
    void publishAll();
    // angle normalization
    double angleNorm(double angle);
    // weight normalization
    void weightNorm();
    // re-sample the particle according to the weights
    void resampling();
    // get the final pose 
    void getFinalPose();
    // gen new particle 
    particle genNewParticle();
};

particle_filter::~particle_filter()
{}

particle_filter::particle_filter(ros::NodeHandle& n):
    n(n)
{
    n.getParam("/particle_filter/init_x", init_x);
    n.getParam("/particle_filter/init_y", init_x);
    n.getParam("/particle_filter/init_theta", init_x);

    n.getParam("/particle_filter/init_rand_xy", init_rand_xy);
    n.getParam("/particle_filter/init_rand_theta", init_rand_theta);

    this->init();

    particles_pub = n.advertise<visualization_msgs::MarkerArray>("particles", 0, true);
    map_sub = n.subscribe("/map", 1, &particle_filter::setMap, this);
    laser_sub = n.subscribe("/course_agv/laser/scan", 1, &particle_filter::doObservation, this);
    odom_sub = n.subscribe("/icp_odom", 1, &particle_filter::doMotion, this);
}

void particle_filter::setMap(nav_msgs::OccupancyGrid input)
{   
    // set once at first time
    if(!isMapSet)
    {   
        cout<<"init the global occupancy grid map"<<endl;
        this->global_map = input;
        isMapSet = true;
    }
}

void particle_filter::init()
{   
    // set state
    state << 0, 0, 0;

    for(int i=0; i<particle_num; i++)
    {   
        particles[i].id = i;
        particles[i].x = init_x + (float(rand()) / float(RAND_MAX)) * 2 * init_rand_xy - init_rand_xy;
        particles[i].y = init_y + (float(rand()) / float(RAND_MAX)) * 2 * init_rand_xy - init_rand_xy;
        particles[i].theta = init_theta + (float(rand()) / float(RAND_MAX)) * 2 * init_rand_theta - init_rand_theta;
        particles[i].weight = float(1/float(particle_num)); // same weight
    }
}

void particle_filter::doMotion(nav_msgs::Odometry input)
{   
    cout<<"doing Motion"<<endl;
    // TODO: Motion Model
}

void particle_filter::doObservation(sensor_msgs::LaserScan input)
{
    cout<<"doing observation"<<endl;
    // TODO: Measurement Model
}

void particle_filter::resampling()
{
    // TODO: Resampling Step
}

void particle_filter::getFinalPose()
{   
    // TODO: Final State Achieve
}

particle particle_filter::genNewParticle()
{
    // TODO: Generate New Particle
}

double particle_filter::angleNorm(double angle)
{
    // 0 ~ 360
    while(angle > 2*M_PI)
        angle = angle - 2*M_PI;
    while(angle < 0)
        angle = angle + 2*M_PI;
    return angle;
}

void particle_filter::publishAll()
{
    visualization_msgs::MarkerArray particle_markers_msg;
    particle_markers_msg.markers.resize(particle_num);

    for(int i=0; i<particle_num; i++)
    {
        particle_markers_msg.markers[i].header.frame_id = "map";
        particle_markers_msg.markers[i].header.stamp = ros::Time::now();
        particle_markers_msg.markers[i].ns = "particle";
        particle_markers_msg.markers[i].id = i;
        particle_markers_msg.markers[i].type = visualization_msgs::Marker::ARROW;
        particle_markers_msg.markers[i].action = visualization_msgs::Marker::ADD;
        particle_markers_msg.markers[i].pose.position.x = particles[i].x;
        particle_markers_msg.markers[i].pose.position.y = particles[i].y;
        particle_markers_msg.markers[i].pose.position.z = 0; // add height for viz ?
        particle_markers_msg.markers[i].pose.orientation.x = 0.0;
        particle_markers_msg.markers[i].pose.orientation.y = 0.0;
        particle_markers_msg.markers[i].pose.orientation.z = sin(particles[i].theta/2);
        particle_markers_msg.markers[i].pose.orientation.w = cos(particles[i].theta/2);
        particle_markers_msg.markers[i].scale.x = 0.1;
        particle_markers_msg.markers[i].scale.y = 0.02;
        particle_markers_msg.markers[i].scale.z = 0.05;
        // particle_markers_msg.markers[i].color.a = particles[i].weight * particle_num / 2; // Don't forget to set the alpha!
       particle_markers_msg.markers[i].color.a = 0.5;
        particle_markers_msg.markers[i].color.r = 1.0;
        particle_markers_msg.markers[i].color.g = 0.0;
        particle_markers_msg.markers[i].color.b = 0.0;
    }
    particles_pub.publish(particle_markers_msg);

    // tf
    geometry_msgs::Quaternion quat_ = tf::createQuaternionMsgFromYaw(state(2));

    geometry_msgs::TransformStamped pf_trans;
    pf_trans.header.stamp = ros::Time::now();
    pf_trans.header.frame_id = "map";
    pf_trans.child_frame_id = "pf_loc";

    pf_trans.transform.translation.x = state(0);
    pf_trans.transform.translation.y = state(1);
    pf_trans.transform.translation.z = 0.0;
    pf_trans.transform.rotation = quat_;
    tf_broadcaster.sendTransform(pf_trans);

    // OR publish others you want

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "particle_filter");
    ros::NodeHandle n;

    particle_filter particle_filter_(n);

    ros::MultiThreadedSpinner spinner(1);
    spinner.spin();

    // ros::spin();

    return 0;
}