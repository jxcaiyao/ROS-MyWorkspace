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

using namespace std;
using namespace Eigen;

class mapping{

public:
    mapping(ros::NodeHandle &n);
	~mapping();
    ros::NodeHandle& n;

    // subers & pubers
    ros::Subscriber laser_sub;
    ros::Publisher map_pub;
    tf::TransformListener listener;
    // transform
    tf::StampedTransform transform;
    // global grid map
    nav_msgs::OccupancyGrid grid_map;
    // some variables
    string world_frame, sensor_frame;
    int map_height, map_width;
    float map_res;
    // grid points location
    MatrixXd grid_points;
    
    // main process
    void process(sensor_msgs::LaserScan input);

    //激光滤波
    sensor_msgs::LaserScan laserFilter(sensor_msgs::LaserScan input);

    int getGridLoc(Eigen::Vector2d loc);

    void updateMap(int index, bool isObstacle);
  
};

mapping::~mapping()
{}

mapping::mapping(ros::NodeHandle& n):
    n(n)
{
    // get the params
    n.getParam("/mapping/world_frame", world_frame);
	n.getParam("/mapping/sensor_frame", sensor_frame);

	n.getParam("/mapping/map_height", map_height);
	n.getParam("/mapping/map_width", map_width);
	n.getParam("/mapping/map_res", map_res);
    
    // iniitialization
    grid_map.info.height = map_height;
    grid_map.info.width = map_width;
    grid_map.info.resolution = map_res;
    grid_map.header.frame_id = world_frame;

    // set origin of map
    grid_map.info.origin.position.x = - float(grid_map.info.width) / 2 * grid_map.info.resolution;
    grid_map.info.origin.position.y = - float(grid_map.info.height) / 2 * grid_map.info.resolution;
    grid_map.info.origin.orientation.w = 1;

    // fill with -1 / unknown in the map
    grid_map.data.assign(map_width * map_height, -1);

    map_pub = n.advertise<nav_msgs::OccupancyGrid>("grid_map_mine", 1);
    laser_sub = n.subscribe("/course_agv/laser/scan", 1, &mapping::process, this);
}

void mapping::process(sensor_msgs::LaserScan input)
{
    cout<<"------seq:  "<<input.header.seq<<endl;

    // transformation is needed
    try{
        listener.lookupTransform(world_frame, sensor_frame,  
                                    ros::Time(0), transform);
    }
    catch(tf::TransformException &ex){
        ROS_ERROR("%s", ex.what());
        return;
    }

    // TODO: Please complete your mapping code
    sensor_msgs::LaserScan laser_extr = this->laserFilter(input);
    int total_num = (laser_extr.angle_max - laser_extr.angle_min) / laser_extr.angle_increment + 1;

    double angle, dist;
    double roll, pitch, yaw;
    Eigen::Vector3d carPose;
    carPose(0) = transform.getOrigin().x();
    carPose(1) = transform.getOrigin().y();
    tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);
    carPose(2) = yaw;

    Eigen::Vector2d startPos, endPos, tmpPos, dPos;
    startPos = carPose.block<2,1>(0,0);

    int startIndex, endIndex, tmpIndex, lastIndex;
    startIndex = this->getGridLoc(startPos);

    double dr = map_res/2;

    for(int i=0; i<total_num; i++){
        angle = laser_extr.angle_min + i * laser_extr.angle_increment;
        dist = laser_extr.ranges[i];
        
        endPos(0) = carPose(0) + dist * cos(carPose(2) + angle);
        endPos(1) = carPose(1) + dist * sin(carPose(2) + angle);
        endIndex = this->getGridLoc(endPos);


        if(dist < 20){
            this->updateMap(endIndex, true);
        }

        double N = floor(dist/dr);
        dPos = (endPos - startPos).normalized() * dr;

        lastIndex = startIndex;
        tmpIndex = startIndex;
        tmpPos = startPos;
        updateMap(tmpIndex, false);

        for(int j=0; j<N; j++){
            tmpPos += dPos;
            tmpIndex = this->getGridLoc(tmpPos);
            if(tmpIndex == lastIndex){
                continue;
            }
            if(tmpIndex == endIndex){
                break;
            }

            this->updateMap(tmpIndex, false);
            lastIndex = tmpIndex;
        }

    }

    // publish
    map_pub.publish(grid_map);
}

sensor_msgs::LaserScan mapping::laserFilter(sensor_msgs::LaserScan input)
{
    sensor_msgs::LaserScan output = input;
    int total_num = (input.angle_max - input.angle_min) / input.angle_increment + 1;

    float dist, sum, n;
    float thread = 0.1;
    for(int i=1; i<total_num-1; i++){
        dist = input.ranges.at(i);
        sum = dist;
        n = 1;
        if(fabs(input.ranges.at(i-1)-dist) < thread){
            sum += input.ranges.at(i-1);
            n++;
        }
        if(fabs(input.ranges.at(i+1)-dist) < thread){
            sum += input.ranges.at(i+1);
            n++;
        }
        output.ranges.at(i) = sum / n;
    } 

    return output;
}

int mapping::getGridLoc(Eigen::Vector2d loc)
{
    int m,n;
    int index;

    m = floor((loc(0) - grid_map.info.origin.position.x) / map_res);
    n = floor((loc(1) - grid_map.info.origin.position.y) / map_res);

    m = min(map_width, max(0, m));
    n = min(map_height, max(0, n));

    index = m + n*map_width;

    return index;
}

void mapping::updateMap(int index, bool isObstacle)
{
    double PA[2] = {0.2, 0.8};
    double PB[2] = {0.8, 0.2};

    if(grid_map.data[index] == -1){
        grid_map.data[index] = 50;
    }

    double p = grid_map.data[index]/100.0;

    grid_map.data[index] = (p*PA[isObstacle])/(p*PA[isObstacle] + (1-p)*PB[isObstacle])*100;
    grid_map.data[index] = (int)min(99, max(1, (int)grid_map.data[index]));
    printf("%d: %d\n",index,grid_map.data[index]);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mapping");
    ros::NodeHandle n;

    mapping mapping_(n);

    ros::MultiThreadedSpinner spinner(1);
    spinner.spin();

    // ros::spin();

    return 0;
}