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
#include <tf2_msgs/TFMessage.h>

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
    float obstacle_std;
    float Neff;
    float gen_var;
    float gen_exp;
    float laser_max_samples;
    Eigen::Vector3d realstatus;

    // subers & pubers
    ros::Subscriber laser_sub;
    ros::Subscriber odom_sub;
    ros::Subscriber map_sub;
    ros::Publisher particles_pub;
    tf::TransformBroadcaster tf_broadcaster;

    ros::Subscriber realtf_sub;

    // particles 
    particle particles[particle_num];

    // global state
    Eigen::Vector3d state;
    // map
    nav_msgs::OccupancyGrid global_map;
    bool isMapSet = false;

    Eigen::Vector3d icp_state;
    bool isIcpSet = false;

    // set map
    void setMap(nav_msgs::OccupancyGrid input);
    // init the state and particles 
    void init();
    // do Motion from ICP odom
    void doMotion(nav_msgs::Odometry input);
    // do Observation to weight each particle
    void doObservation(sensor_msgs::LaserScan input);
    // calculate possibility of being a obstacle
    double calPossibilily(Eigen::Vector2d laser_pos);
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
    particle genNewParticle(particle particle_root, int id);

    void filter();

    void realtf_subfunc(tf2_msgs::TFMessage realtf);
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
    n.getParam("/particle_filter/obstacle_std", obstacle_std);
    n.getParam("/particle_filter/Neff", Neff);
    n.getParam("/particle_filter/gen_var", gen_var);
    n.getParam("/particle_filter/gen_exp", gen_exp);
    n.getParam("/particle_filter/laser_max_samples", laser_max_samples);

    this->init();
    ROS_INFO("doing initialization");
    particles_pub = n.advertise<visualization_msgs::MarkerArray>("particles", 0, true);
    map_sub = n.subscribe("/map", 1, &particle_filter::setMap, this);
    laser_sub = n.subscribe("/course_agv/laser/scan", 1, &particle_filter::doObservation, this);
    odom_sub = n.subscribe("/icp_odom", 1, &particle_filter::doMotion, this);
    realtf_sub = n.subscribe("/tf", 1, &particle_filter::realtf_subfunc, this);
}

void particle_filter::setMap(nav_msgs::OccupancyGrid input)
{   
    // set once at first time
    if(!isMapSet)
    {   
        cout<<"init the global occupancy grid map"<<endl;
        this->global_map = input;
        isMapSet = true;

        nav_msgs::OccupancyGrid grid_map = global_map;

        for(int i=1; i<global_map.info.height-1; i++){
            for(int j=1; j<global_map.info.width-1; j++){
                if(global_map.data[i*global_map.info.width + j] >= 80){
                    if(    global_map.data[(i+1)*global_map.info.width + j-1]   > 80
                        && global_map.data[(i+1)*global_map.info.width + j+0]   > 80
                        && global_map.data[(i+1)*global_map.info.width + j+1]   > 80
                        && global_map.data[(i+0)*global_map.info.width + j-1]   > 80
                        && global_map.data[(i+0)*global_map.info.width + j+1]   > 80
                        && global_map.data[(i-1)*global_map.info.width + j-1]   > 80
                        && global_map.data[(i-1)*global_map.info.width + j+0]   > 80
                        && global_map.data[(i-1)*global_map.info.width + j+1]   > 80){
                        grid_map.data[i*global_map.info.width + j] = 0;
                    }
                }
            }
        }
        for(int i=0; i<grid_map.info.height; i++){
            grid_map.data[i*grid_map.info.width] = 0;
            grid_map.data[i*grid_map.info.width + grid_map.info.width-1] = 0;
        }
        for(int j=0; j<grid_map.info.width; j++){
            grid_map.data[j] = 0;
            grid_map.data[(grid_map.info.height-1)*grid_map.info.width + j] = 0;
        }

        global_map = grid_map;
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
        particles[i].theta = angleNorm(particles[i].theta);
        particles[i].weight = float(1/float(particle_num)); // same weight
    }
}

void particle_filter::doMotion(nav_msgs::Odometry input)
{   
    cout<<"doing Motion"<<endl;
    // TODO: Motion Model

    Eigen::Vector2d dr;
    Eigen::Matrix2d R;
    Eigen::Vector3d mu;

    double roll, pitch, yaw;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(input.pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    if(!isIcpSet){
        icp_state << input.pose.pose.position.x, input.pose.pose.position.y, yaw;
        isIcpSet = true;
        return;
    }else{
        dr << input.pose.pose.position.x - icp_state(0), input.pose.pose.position.y - icp_state(1);
        R << cos(icp_state(2)), sin(icp_state(2)),
            -sin(icp_state(2)), cos(icp_state(2));
        mu.block<2,1>(0,0) = R * dr;
        mu(2) = angleNorm(yaw - icp_state(2));

        icp_state(0) = input.pose.pose.position.x;
        icp_state(1) = input.pose.pose.position.y;
        icp_state(2) = yaw;
    }

    for(int i=0; i<particle_num; i++){
        R << cos(particles[i].theta), -sin(particles[i].theta),
             sin(particles[i].theta),  cos(particles[i].theta);
        Eigen::Vector3d tmu = mu;

        tmu(0) += 0.1 * 2*(1.0*rand() / RAND_MAX - 0.5);
        tmu(1) += 0.05 * 2*(1.0*rand() / RAND_MAX - 0.5);
        tmu(2) += 0.1 * 2*(1.0*rand() / RAND_MAX - 0.5);
        dr = R * tmu.block<2,1>(0,0);

        particles[i].x += dr(0);
        particles[i].y += dr(1);
        particles[i].theta += tmu(2);
        particles[i].theta = angleNorm(particles[i].theta);
    }
}

void particle_filter::doObservation(sensor_msgs::LaserScan input)
{
    // cout<<"doing observation"<<endl;
    // TODO: Measurement Model

    // 激光采样
    Eigen::MatrixXd laser_smp;

    int input_num = (input.angle_max - input.angle_min) / input.angle_increment + 1;
    if(input_num > laser_max_samples){
        double findex, dindex;
        dindex = input_num / laser_max_samples;
        findex = 0;
        laser_smp = Eigen::MatrixXd::Zero(2,laser_max_samples);
        for(int i=0; i<laser_max_samples; i++){
            laser_smp(0,i) = input.ranges.at(round(findex));
            laser_smp(1,i) = input.angle_min + round(findex) * input.angle_increment;
            findex += dindex;
        }
        input_num = laser_max_samples;
    }else{
        for(int i=0; i<input_num; i++){
            laser_smp(0,i) = input.ranges.at(i);
            laser_smp(1,i) = input.angle_min + i * input.angle_increment;
        }
    }

    // 改变粒子权重
    Eigen::Vector2d laser_pos;
    double weight;
    for(int i=0; i<particle_num; i++){
        weight = 0;
        for(int j=0; j<input_num; j++){
            laser_pos(0) = laser_smp(0,j) * cos(laser_smp(1,j) + particles[i].theta) + particles[i].x;
            laser_pos(1) = laser_smp(0,j) * sin(laser_smp(1,j) + particles[i].theta) + particles[i].y;
            weight += calPossibilily(laser_pos);
        }
        particles[i].weight = weight;
        // cout << "weight: " << weight << endl;
    }

    // 归一化
    weightNorm();

    // 计算期望位姿
    getFinalPose();

    // 输出结果
    publishAll();

    // 重采样
    resampling();

    // 去除离散点
    filter();
}

double particle_filter::calPossibilily(Eigen::Vector2d laser_pos)
{
    int minindex = -1;
    double mindist = 1e10;
    double tmpdist;
    int grid_num = global_map.info.width * global_map.info.height;
    double res = global_map.info.resolution;

    Eigen::Vector2d origin;
    origin(0) = global_map.info.origin.position.x;
    origin(1) = global_map.info.origin.position.y;

    int m0,n0;
    m0 = (laser_pos - origin)(0) / res;
    n0 = (laser_pos - origin)(1) / res;

    if(m0 < 0 || m0 > global_map.info.width || n0 < 0 || n0 > global_map.info.height){
        return 0;
    }

    Eigen::Vector2d grid_pos;
    int m,n;

    for(int i=0; i<grid_num; i++){
        if(global_map.data.at(i) < 80){
            continue;
        }

        n = i / global_map.info.width;
        m = i - n * global_map.info.width;

        if(sqrt((m-m0)*(m-m0) + (n-n0)*(n-n0)) > 9.0 / res / res){
            continue;
        }

        grid_pos << m*res + 0.5*res, 
                    n*res + 0.5*res;
        grid_pos += origin;

        tmpdist = (grid_pos - laser_pos).norm();

        if(tmpdist < mindist){
            mindist = tmpdist;
            minindex = i;
        }
    }

    double ret;

    if(minindex != -1){
        ret = exp(-0.5*mindist*mindist/(obstacle_std*obstacle_std));
    }else{
        ret = 0;
    }
    // cout << "mindist: " << mindist << endl;
    // cout << "ret: " << ret << endl;
    return ret;
}

void particle_filter::weightNorm()
{
    double sum_weight = 0;
    for(int i=0; i<particle_num; i++){
        sum_weight += particles[i].weight;
    }

    for(int i=0; i<particle_num; i++){
        particles[i].weight /= sum_weight;
        // cout << "weight: " << particles[i].weight << endl;
        // cout << "sum_weight: " << sum_weight << endl;
    }
}

void particle_filter::resampling()
{
    // TODO: Resampling Step
    double sum_weight = 0;
    double sum_pos = 0;
    for(int i=0; i<particle_num; i++){
        sum_weight += particles[i].weight;
        if(particles[i].weight > Neff){
            sum_pos++;
        }
    }

    particle tmp_particles[particle_num];

    cout << "sum_weight: " << sum_weight << endl;
    if(sum_pos > 0){
        double r = float(rand()) / float(RAND_MAX) * sum_weight / particle_num;
        // cout << "r: " << r << endl;
        for(int i=0; i<particle_num; i++){
            tmp_particles[i] = particles[i];
        }

        double weight = 0;
        int i=-1;
        int j=-1;
        while(i < particle_num){
            if(weight < r){
                j++;
                weight += tmp_particles[j].weight;
            }else{
                i++;
                r += sum_weight / particle_num;
                
                particles[i] = genNewParticle(tmp_particles[j], i);
            }
        }
        if(j > particle_num){
            printf("wrong resample! j=%d\n", j);
        }
    }
}

void particle_filter::getFinalPose()
{   
    // TODO: Final State Achieve

    Eigen::Vector3d avg_state = Eigen::Vector3d::Zero(3,1);
    Eigen::Vector3d tmp_vec;

    for(int i=0; i<particle_num; i++)
    {   
        tmp_vec << particles[i].x, particles[i].y, particles[i].theta; 
        avg_state += tmp_vec * particles[i].weight;
    }

    state = avg_state;
}

particle particle_filter::genNewParticle(particle particle_root, int id)
{
    // TODO: Generate New Particle
    particle particle_ret;
    double tmp = exp(-pow(particle_root.weight, 2) / gen_var);
    particle_ret.id = id;
    particle_ret.x = particle_root.x + 
        2 * (float(rand()) / float(RAND_MAX)-0.5) * 
        gen_exp * tmp;
    particle_ret.y = particle_root.y + 
        2 * (float(rand()) / float(RAND_MAX)-0.5) * 
        gen_exp * tmp;
    particle_ret.theta = particle_root.theta + 
        2 * (float(rand()) / float(RAND_MAX)-0.5) * 
        gen_exp / 3 * tmp;
    particle_ret.weight = particle_root.weight;

    return particle_ret;
}

double particle_filter::angleNorm(double angle)
{
    // -180 ~ 180
    while(angle > M_PI)
        angle = angle - 2*M_PI;
    while(angle < -M_PI)
        angle = angle + 2*M_PI;
    return angle;
}

void particle_filter::filter()
{
    double Ex, Ey, Ex2, Ey2;
    double Sx, Sy;

    Ex = Ey = Ex2 = Ey2 = 0;
    for(int i=0; i<particle_num; i++){
        Ex += particles[i].x;
        Ex2 += particles[i].x * particles[i].x;
        Ey += particles[i].y;
        Ey2 += particles[i].y * particles[i].y;
    }
    Ex /= particle_num;
    Ex2 /= particle_num;
    Ey /= particle_num;
    Ey2 /= particle_num;

    Sx = sqrt(Ex2 - Ex*Ex);
    Sy = sqrt(Ey2 - Ey*Ey);

    for(int i=0; i<particle_num; i++){
        if(fabs(particles[i].x - Ex) > 3*Sx || fabs(particles[i].y - Ey) > 3*Sy){
            if(i > 0){
                particles[i] = genNewParticle(particles[i-1], i);
            }
        }
    }
}

void particle_filter::publishAll()
{
    // ROS_INFO("publish all");
    visualization_msgs::MarkerArray particle_markers_msg;
    particle_markers_msg.markers.resize(particle_num);

    for(int i=0; i<particle_num; i++)
    {
        particle_markers_msg.markers[i].header.frame_id = "world_base";
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
        particle_markers_msg.markers[i].color.a = particles[i].weight * particle_num / 2; // Don't forget to set the alpha!
        // particle_markers_msg.markers[i].color.a = 0.5;
        particle_markers_msg.markers[i].color.r = 1.0;
        particle_markers_msg.markers[i].color.g = 0.0;
        particle_markers_msg.markers[i].color.b = 0.0;
    }
    particles_pub.publish(particle_markers_msg);

    // tf
    geometry_msgs::Quaternion quat_ = tf::createQuaternionMsgFromYaw(state(2));

    geometry_msgs::TransformStamped pf_trans;
    pf_trans.header.stamp = ros::Time::now();
    pf_trans.header.frame_id = "world_base";
    pf_trans.child_frame_id = "pf_loc";

    pf_trans.transform.translation.x = state(0);
    pf_trans.transform.translation.y = state(1);
    pf_trans.transform.translation.z = 0.0;
    pf_trans.transform.rotation = quat_;
    tf_broadcaster.sendTransform(pf_trans);

    // OR publish others you want
    FILE *fp;
    fp = fopen("/home/zailu/icp_err_data_normal.dat", "a+");
    if(fp == NULL){
        cout << "tf open wrong!" << endl;
    }else{
        fprintf(fp, "%.3f,%.3f,%.3f\n", state(0)-realstatus(0), state(1)-realstatus(1), state(2)-realstatus(2));
    }
    fclose(fp);
}

void particle_filter::realtf_subfunc(tf2_msgs::TFMessage realtf)
{
    double roll, pitch, yaw;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(realtf.transforms.at(0).transform.rotation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    realstatus <<   realtf.transforms.at(0).transform.translation.x, 
                    realtf.transforms.at(0).transform.translation.y,
                    yaw;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "particle_filter");
    ros::NodeHandle n;

    ROS_INFO("here1");
    particle_filter particle_filter_(n);
    ROS_INFO("here2");

    ros::MultiThreadedSpinner spinner(1);
    spinner.spin();

    // ros::spin();

    return 0;
}