#include "ros/ros.h"
#include "ros/console.h"
#include <stdio.h>
#include <Eigen/Eigen>

#include "ros/publisher.h"
#include "ros/subscriber.h"
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;
using namespace Eigen;

// structure of the LandMark set, contians the landMarks 
typedef struct{
    std::vector<float> position_x;
    std::vector<float> position_y;
    std::vector<int> id;
} LandMarkSet;

class extraction{

public:
	extraction(ros::NodeHandle &n);
	~extraction();
    ros::NodeHandle& n;
 
    // get the clusters
    float range_threshold;
    // filter for landmarks extractions
    float radius_max_th;
    // filter for landmarks extractions
    int landMark_min_pt;

    // listen the ros::laserScan
    ros::Subscriber laser_sub;
    // publish the landMarks as ros::Markers
    ros::Publisher landMark_pub;

    // main process
    void process(sensor_msgs::LaserScan input);
    // filter & extraction process
    LandMarkSet extractLandMark(sensor_msgs::LaserScan input);
    // publish the landMarks
    void publishLandMark(LandMarkSet input);
    // 2D euclidean distance calculation
    float calc_dist(const Eigen::Vector2d &pta, const Eigen::Vector2d &ptb);
};

extraction::~extraction()
{}

extraction::extraction(ros::NodeHandle& n):
    n(n)
{   
    // get the params
    n.getParam("/extraction/range_threshold", range_threshold);
    n.getParam("/extraction/radius_max_th", radius_max_th);
    n.getParam("/extraction/landMark_min_pt", landMark_min_pt);
    
    landMark_pub = n.advertise<visualization_msgs::MarkerArray>("landMarks", 1);
    laser_sub = n.subscribe("/course_agv/laser/scan", 1, &extraction::process, this);
}

void extraction::process(sensor_msgs::LaserScan input)
{   
    double time_0 = (double)ros::Time::now().toSec();

    int label = 0;
    sensor_msgs::LaserScan laser_extr = input;

    int total_num = (input.angle_max - input.angle_min) / input.angle_increment + 1;

    // init the previous/last point
    Vector2d last_xy(input.ranges[0] * std::cos(input.angle_min), input.ranges[0] * std::sin(input.angle_min));

    // set the intensities as labels
    Vector2d curr_xy;
    float angle, delta_dis;
    for(int i=0; i<total_num; i++)
    {   
        angle = input.angle_min + i * input.angle_increment;
        curr_xy << input.ranges[i] * std::cos(angle), input.ranges[i] * std::sin(angle);
        
        delta_dis = this->calc_dist(curr_xy, last_xy);

        if(delta_dis > range_threshold)
            label++;

        laser_extr.intensities[i] = label;

        last_xy = curr_xy;
    }

    cout<<"Total original labels: "<<label<<endl;

    LandMarkSet landmarks_ = this->extractLandMark(laser_extr);

    this->publishLandMark(landmarks_);

    double time_1 = (double)ros::Time::now().toSec();
    cout<<"time_cost:  "<<time_1-time_0<<endl;
}

LandMarkSet extraction::extractLandMark(sensor_msgs::LaserScan input)
{   
    int total_num = (input.angle_max - input.angle_min) / input.angle_increment + 1;

    LandMarkSet landMarks;

    // TODO: please code by yourself
    int landMark_id = 0;
    int cur_label = 0;
    int begin_index = 0;
    int end_index = 0;
    int mid_index = 0;
    Vector2d begin_xy;
    Vector2d end_xy;
    Vector2d mid_xy;
    Vector2d center_xy;
    Vector2d center2_xy;
    float angle, dis;
    for(int i=0; i<total_num; i++){
        if(cur_label != input.intensities[i]){
            end_index = i-1;
            if(end_index - begin_index >= this->landMark_min_pt){
                angle = input.angle_min + begin_index * input.angle_increment;
                begin_xy << input.ranges[begin_index] * std::cos(angle), input.ranges[begin_index] * std::sin(angle);
                angle = input.angle_min + end_index * input.angle_increment;
                end_xy << input.ranges[end_index] * std::cos(angle), input.ranges[end_index] * std::sin(angle);
                dis = this->calc_dist(begin_xy, end_xy);
                if(dis <= radius_max_th){
                    mid_index = (int)((begin_index + end_index)/2.0 + 0.5);
                    angle = input.angle_min + mid_index * input.angle_increment;
                    mid_xy << input.ranges[mid_index] * std::cos(angle), input.ranges[mid_index] * std::sin(angle);

                    double R = 0.09;
                    center_xy = mid_xy;
                    center_xy += R * center_xy.normalized();

                    center2_xy = (begin_xy + end_xy) / 2;
                    if(dis < 2*R){
                        center2_xy += sqrt(R*R - dis*dis/4) * center2_xy.normalized();
                    }
                    center_xy = (center_xy + center2_xy) / 2;

                    std::cout << landMark_id << ": " << center_xy.transpose() << endl;

                    landMarks.id.push_back(landMark_id);
                    landMarks.position_x.push_back(center_xy.x());
                    landMarks.position_y.push_back(center_xy.y());
                    landMark_id++;
                }
            }
            begin_index = i;
            cur_label = input.intensities[i];
        }
    }

    return landMarks;
}

void extraction::publishLandMark(LandMarkSet input)
{
    if(input.id.size() <= 0)
        return;

    visualization_msgs::MarkerArray landMark_array_msg;

    landMark_array_msg.markers.resize(input.id.size());

    for(int i=0; i<input.id.size(); i++)
    {
        landMark_array_msg.markers[i].header.frame_id = "course_agv__hokuyo__link";
        landMark_array_msg.markers[i].header.stamp = ros::Time(0);
        landMark_array_msg.markers[i].ns = "lm";
        landMark_array_msg.markers[i].id = i;
        landMark_array_msg.markers[i].type = visualization_msgs::Marker::SPHERE;
        landMark_array_msg.markers[i].action = visualization_msgs::Marker::ADD;
        landMark_array_msg.markers[i].pose.position.x = input.position_x.at(i);
        landMark_array_msg.markers[i].pose.position.y = input.position_y.at(i);
        landMark_array_msg.markers[i].pose.position.z = 0; // 2D
        landMark_array_msg.markers[i].pose.orientation.x = 0.0;
        landMark_array_msg.markers[i].pose.orientation.y = 0.0;
        landMark_array_msg.markers[i].pose.orientation.z = 0.0;
        landMark_array_msg.markers[i].pose.orientation.w = 1.0;
        landMark_array_msg.markers[i].scale.x = 0.2;
        landMark_array_msg.markers[i].scale.y = 0.2;
        landMark_array_msg.markers[i].scale.z = 0.2;
        landMark_array_msg.markers[i].color.a = 0.5; // Don't forget to set the alpha!
        landMark_array_msg.markers[i].color.r = 0.0;
        landMark_array_msg.markers[i].color.g = 0.0;
        landMark_array_msg.markers[i].color.b = 1.0;
    }

    landMark_pub.publish(landMark_array_msg);
}

float extraction::calc_dist(const Eigen::Vector2d &pta, const Eigen::Vector2d &ptb)
{
    // please code by yourself
    float dist;
    dist = (pta - ptb).norm();
    return dist;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "extraction");
    ros::NodeHandle n;

    extraction extraction_(n);

    ros::MultiThreadedSpinner spinner(1);
    spinner.spin();

    return 0;
}