#include "ros/ros.h"
#include "ros/console.h"
#include <stdio.h>

#include <numeric>
#include <vector>
#include <Eigen/Eigen>

#include "ros/publisher.h"
#include "ros/subscriber.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>

using namespace std;
using namespace Eigen;

// structure of the nearest neighbor 
typedef struct{
    std::vector<float> distances;
    std::vector<int> src_indices;
    std::vector<int> tar_indices;
} NeighBor;

class icp{

public:

    icp(ros::NodeHandle &n);
    ~icp();
    ros::NodeHandle& n;

    // robot init states
    double robot_x;
    double robot_y;
    double robot_theta;
    // sensor states = robot_x_y_theta
    Vector3d sensor_sta;

    // max iterations
    int max_iter;
    // distance threshold for filter the matching points
    double dis_th;
    // tolerance to stop icp
    double tolerance;
    // if is the first scan, set as the map/target
    bool isFirstScan;
    // src point cloud matrix
    MatrixXd src_pc;
    // target point cloud matrix
    MatrixXd tar_pc;

    // ICP process function
    void process(sensor_msgs::LaserScan input);
    // transform the ros msg to Eigen Matrix
    Eigen::MatrixXd rosmsgToEigen(const sensor_msgs::LaserScan input);
    // fint the nearest points & filter
    NeighBor findNearest(const Eigen::MatrixXd &src, const Eigen::MatrixXd &tar);
    // get the transform from two point sets in one iteration
    Eigen::Matrix3d getTransform(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B);
    // calc 2D Euclidean distance
    float calc_dist(const Eigen::Vector2d &pta, const Eigen::Vector2d &ptb);
    // transform vector states to matrix form
    Eigen::Matrix3d staToMatrix(const Vector3d sta);
  
    // ros-related subscribers, publishers and broadcasters
    ros::Subscriber laser_sub;
    void publishResult(Matrix3d T);
 	tf::TransformBroadcaster odom_broadcaster;
 	ros::Publisher odom_pub;
};

icp::~icp()
{}

icp::icp(ros::NodeHandle& n):
    n(n)
{	

	// get the params
	n.getParam("/icp/robot_x", robot_x);
	n.getParam("/icp/robot_y", robot_y);
	n.getParam("/icp/robot_theta", robot_theta);
	sensor_sta << robot_x, robot_y, robot_theta;

	n.getParam("/icp/max_iter", max_iter);
	n.getParam("/icp/tolerance", tolerance);
	n.getParam("/icp/dis_th", dis_th);

    isFirstScan = true;
    laser_sub = n.subscribe("/course_agv/laser/scan", 1, &icp::process, this);
    odom_pub = n.advertise<nav_msgs::Odometry>("icp_odom", 1);
}

void icp::process(sensor_msgs::LaserScan input)
{
    cout<<"------seq:  "<<input.header.seq<<endl;

    // set the inital
    if(isFirstScan)
    {
        tar_pc = this->rosmsgToEigen(input);
        isFirstScan =false;
        return;
    }

	double time_0 = (double)ros::Time::now().toSec();

    // init some variables
    double cols;
    NeighBor neigh;
    double mean_dist = 0;
    double last_dist = 0;
    MatrixXd src_pc_rearr;
    MatrixXd tar_pc_rearr;
    
    Eigen::Matrix3d Transform_acc = Eigen::MatrixXd::Identity(3,3);
    src_pc = this->rosmsgToEigen(input);

    // TODO: preprocess src_pc

    // main LOOP
    for(int i=0; i<max_iter; i++)
    {	
    	// please code by yourself
        neigh = findNearest(Transform_acc * src_pc,tar_pc);
        if(neigh.distances.size() == 0){
            cout << "no solution!" << endl;
            break;
        }

        cols = neigh.src_indices.size();
        src_pc_rearr = MatrixXd::Zero(2,cols);
        tar_pc_rearr = MatrixXd::Zero(2,cols);
        for(int j=0; j<cols; j++){
            src_pc_rearr.block<2,1>(0,j) = src_pc.block<2,1>(0,neigh.src_indices[j]);
            tar_pc_rearr.block<2,1>(0,j) = tar_pc.block<2,1>(0,neigh.tar_indices[j]);
        }
        // cout << "here1" << endl;
        Transform_acc = getTransform(src_pc_rearr, tar_pc_rearr);

        // cout << "here2" << endl;
        mean_dist = std::accumulate(neigh.distances.begin(), neigh.distances.end(), 0.0) / neigh.distances.size();
        if(abs(mean_dist - last_dist) < this->tolerance){
            // cout << "cols: " << src_pc_rearr.cols() << endl;
            // cout << "mean_dist:  " << mean_dist << endl;
            cout << "iter times:  " << i+1 << endl;
            break;
        }
        last_dist = mean_dist;
        // cout << "here3" << endl;
    }
    cout << "cols: " << src_pc_rearr.cols() << endl;
    cout << "mean_dist:  " << mean_dist << endl;
    cout << "T:" << endl;
    cout << Transform_acc << endl;

    tar_pc = this->rosmsgToEigen(input);

    this->publishResult(Transform_acc);

	double time_1 = (double)ros::Time::now().toSec();
	cout<<"time_cost:  "<<time_1-time_0<<endl<<endl;
}

Eigen::MatrixXd icp::rosmsgToEigen(const sensor_msgs::LaserScan input)
{
    int total_num = (input.angle_max - input.angle_min) / input.angle_increment + 1;

    Eigen::MatrixXd pc = Eigen::MatrixXd::Ones(3,total_num);

    float angle;
    for(int i=0; i<total_num; i++)
    {
        angle = input.angle_min + i * input.angle_increment;

        pc(0,i) = input.ranges[i] * std::cos(angle);
        pc(1,i) = input.ranges[i] * std::sin(angle);
    }
    return pc;
}

NeighBor icp::findNearest(const Eigen::MatrixXd &src, const Eigen::MatrixXd &tar)
{
    // please code by yourself
    double dist;
    double min_dist;
    int min_index;
    Vector2d src_vec;
    Vector2d tar_vec;
    NeighBor neigh;
    // cout << "here1" << endl;

    for(int i=0; i<src.cols(); i++){
        src_vec = src.block<2,1>(0,i);
        min_index = 0;
        tar_vec = tar.block<2,1>(0,min_index);
        min_dist = calc_dist(src_vec, tar_vec);
        for(int j=0; j<tar.cols(); j++){
            tar_vec = tar.block<2,1>(0,j);
            dist = calc_dist(src_vec, tar_vec);
            if(dist < min_dist){
                min_index = j;
                min_dist = dist;
            }
        }
        if(min_dist < dis_th){
            neigh.distances.push_back(min_dist);
            neigh.src_indices.push_back(i);
            neigh.tar_indices.push_back(min_index);
        }
        // cout << "min_dist: " << min_dist << endl;
    }

    // cout << "here2" << endl;
    return neigh;
}

Eigen::Matrix3d icp::getTransform(const Eigen::MatrixXd &src, const Eigen::MatrixXd &tar)
{
    // please code by yourself
    Matrix3d T = Matrix3d::Identity(3,3);

    int cols = src.cols();
    // cout << "cols: " << cols << endl;
    Vector2d src_mc = Vector2d::Zero();
    Vector2d tar_mc = Vector2d::Zero();
    MatrixXd src_q = src;
    MatrixXd tar_q = tar;

    for(int i=0; i<cols; i++){
        src_mc = src_mc + src.block<2,1>(0,i);
        tar_mc = tar_mc + tar.block<2,1>(0,i);
    }
    src_mc = src_mc / cols;
    tar_mc = tar_mc / cols;

    for(int i=0; i<cols; i++){
        src_q.block<2,1>(0,i) = src.block<2,1>(0,i) - src_mc;
        tar_q.block<2,1>(0,i) = tar.block<2,1>(0,i) - tar_mc;
    }

    MatrixXd W = tar_q * src_q.transpose();
    MatrixXd U;
    MatrixXd V;
    Matrix2d R;
    Vector2d t;

    // cout << "here1" << endl;
    // cout << "W: " << endl;
    // cout << W << endl;
    JacobiSVD<MatrixXd> svd(W, ComputeFullU | ComputeFullV);
    // cout << "here2" << endl;
    U = svd.matrixU();
    V = svd.matrixV();
    // cout << "here3" << endl;
    // cout << "U: " << endl;
    // cout << U << endl;
    // cout << "V: " << endl;
    // cout << V << endl;
    R = V * U.transpose();
    // cout << "here4" << endl;

    t = tar_mc - R * src_mc;
    // cout << "here5" << endl;

    T.block<2,2>(0,0) = R;
    T.block<2,1>(0,2) = t;
    // cout << "here6" << endl;
    // cout << "T: " << endl;
    // cout << T << endl;

    return T;
}

float icp::calc_dist(const Eigen::Vector2d &pta, const Eigen::Vector2d &ptb)
{
    // please code by yourself
    float dist;
    dist = (pta-ptb).norm();
    // dist = sqrt(pow(pta(0)-ptb(0),2)+pow(pta(1)-ptb(1),2));
    // cout << "dist: " << dist << endl;
    return dist;
}

Eigen::Matrix3d icp::staToMatrix(Eigen::Vector3d sta)
{
	Matrix3d RT;
    RT << cos(sta(2)), -sin(sta(2)), sta(0),
          sin(sta(2)), cos(sta(2)),  sta(1),
          0,           0,            1;
    return RT;
}

void icp::publishResult(Eigen::Matrix3d T)
{	
    float delta_yaw = atan2(T(1,0), T(0,0));
    cout<<"sensor-delta-xyt: "<<T(0,2)<<" "<<T(1,2)<<" "<<delta_yaw<<endl;

    sensor_sta(0) = sensor_sta(0) + cos(sensor_sta(2))*T(0,2) - sin(sensor_sta(2))*T(1,2);
    sensor_sta(1) = sensor_sta(1) + sin(sensor_sta(2))*T(0,2) + cos(sensor_sta(2))*T(1,2);
    sensor_sta(2) = sensor_sta(2) + delta_yaw;

    cout<<"sensor-global: "<<sensor_sta.transpose()<<endl;

    // tf
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(sensor_sta(2));

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = "world_base";
    odom_trans.child_frame_id = "icp_odom";

    odom_trans.transform.translation.x = sensor_sta(0);
    odom_trans.transform.translation.y = sensor_sta(1);
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    odom_broadcaster.sendTransform(odom_trans);

    // odom
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "world_base";

    odom.pose.pose.position.x = sensor_sta(0);
    odom.pose.pose.position.y = sensor_sta(1);
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    odom_pub.publish(odom);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "icp");
    ros::NodeHandle n;

    icp icp_(n);

    ros::MultiThreadedSpinner spinner(1);
    spinner.spin();

    return 0;
}