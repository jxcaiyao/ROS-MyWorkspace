#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/Float64.h"
#include "rosgraph_msgs/Clock.h"
#include <stdio.h>

#include <numeric>
#include <vector>
#include <Eigen/Eigen>

#include "ros/publisher.h"
#include "ros/subscriber.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;
using namespace Eigen;

typedef struct{
    std::vector<float> distances;
    std::vector<int> src_indices;
    std::vector<int> tar_indices;
} NeighBor;

class icp_lm{

public:

    icp_lm(ros::NodeHandle &n);
    ~icp_lm();
    ros::NodeHandle& n;

    // robot init states
    double robot_x;
    double robot_y;
    double robot_theta;

    double robot_dx;
    double robot_dy;
    double robot_dtheta;
    double v_left;
    double v_right;
    double tread;
    double time_last;
    double time_now;
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

    bool isFirstClock;
    // src point cloud matrix
    MatrixXd src_pc;
    // target point cloud matrix
    MatrixXd tar_pc;
    // min match_cnt
    int min_match;

    // main process
    void process(visualization_msgs::MarkerArray input);

    void v_left_subfunc(std_msgs::Float64 v_left_msgs);
    void v_right_subfunc(std_msgs::Float64 v_right_msgs);
    void clock_subfunc(rosgraph_msgs::Clock clock_msgs);
    // landMarks to Eigen::Matrix
    Eigen::MatrixXd landMarksToMatrix(visualization_msgs::MarkerArray input);
    // fint the nearest points & filter
    NeighBor findNearest(const Eigen::MatrixXd &src, const Eigen::MatrixXd &tar);

    Eigen::Matrix3d getWheelTransform(void);    
    // get the transform from two point sets in one iteration
    Eigen::Matrix3d getTransform(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B);
    // calc 2D Euclidean distance
    float calc_dist(const Eigen::Vector2d &pta, const Eigen::Vector2d &ptb);
    // transform vector states to matrix form
    Eigen::Matrix3d staToMatrix(const Vector3d sta);

    // ros-related subscribers, publishers and broadcasters
    ros::Subscriber landMark_sub;
    ros::Subscriber v_left_sub;
    ros::Subscriber v_right_sub;
    ros::Subscriber clock_sub;
    void publishResult(Matrix3d T);
 	tf::TransformBroadcaster odom_broadcaster;
 	ros::Publisher odom_pub;
};

icp_lm::~icp_lm()
{}

icp_lm::icp_lm(ros::NodeHandle& n):
    n(n)
{
    // get the params
	n.getParam("/icp_lm/robot_x", robot_x);
	n.getParam("/icp_lm/robot_y", robot_y);
	n.getParam("/icp_lm/robot_theta", robot_theta);
	sensor_sta << robot_x, robot_y, robot_theta;

    n.getParam("/icp_lm/max_iter", max_iter);
	n.getParam("/icp_lm/tolerance", tolerance);
	n.getParam("/icp_lm/dis_th", dis_th);
    n.getParam("/icp_lm/min_match", min_match);

    isFirstScan = true;
    isFirstClock = true;

    robot_dx = 0;
    robot_dy = 0;
    robot_dtheta = 0;
    v_left = 0;
    v_right = 0;

    tread = 2*(0.1+0.08/6);
    // time_now = (double)ros::Time::now().toSec();
    // time_last = time_now;

    landMark_sub = n.subscribe("/landMarks", 1, &icp_lm::process, this);
    
    v_left_sub = n.subscribe("/course_agv/left_wheel_velocity_controller/command", 
                                1, &icp_lm::v_left_subfunc, this);
    v_right_sub = n.subscribe("/course_agv/right_wheel_velocity_controller/command", 
                                1, &icp_lm::v_right_subfunc, this);
    clock_sub = n.subscribe("/clock",1,&icp_lm::clock_subfunc,this);
    
    odom_pub = n.advertise<nav_msgs::Odometry>("icp_odom", 1);
}

void icp_lm::process(visualization_msgs::MarkerArray input)
{   
    // cout<<"------Time:  "<<input.markers[0].header.stamp<<endl;
    
    double time_0 = (double)ros::Time::now().toSec();

    if(isFirstScan)
    {
        tar_pc = this->landMarksToMatrix(input);
        isFirstScan =false;
        return;
    }

    // init some variables
    double cols;
    NeighBor neigh;
    double mean_dist = 0;
    double last_dist = 0;
    MatrixXd src_pc_2D;
    MatrixXd tar_pc_2D;
    MatrixXd src_pc_rearr;
    MatrixXd tar_pc_rearr;
    
    Eigen::Matrix3d T = Eigen::MatrixXd::Identity(3,3);
    Eigen::Matrix3d Transform_acc = Eigen::MatrixXd::Identity(3,3);
    src_pc = this->landMarksToMatrix(input);

    // preprocess src_pc_2D
    
    src_pc_2D = MatrixXd::Zero(2,src_pc.cols());
    for(int i=0; i<src_pc.cols(); i++){
        src_pc_2D.block<2,1>(0,i) = src_pc.block<2,1>(0,i);
    }
    tar_pc_2D = MatrixXd::Zero(2,tar_pc.cols());
    for(int i=0; i<tar_pc.cols(); i++){
        tar_pc_2D.block<2,1>(0,i) = tar_pc.block<2,1>(0,i);
    }
    T = this->getWheelTransform();
    // std::cout << "T: " << endl << T << endl;
    src_pc_2D = T.block<2,2>(0,0) * src_pc_2D + T.block<2,1>(0,2) * MatrixXd::Ones(1,src_pc_2D.cols());
    Transform_acc = T * Transform_acc;

    // main LOOP
    for(int i=0; i<max_iter; i++)
    {
        // please code by yourself
        neigh = this->findNearest(src_pc_2D, tar_pc_2D);
        if(neigh.distances.size() < min_match){
            std::cout << "no solution!" << endl;
            break;
        }

        cols = neigh.distances.size();
        src_pc_rearr = MatrixXd::Zero(2,cols);
        tar_pc_rearr = MatrixXd::Zero(2,cols);
        for(int j=0; j<cols; j++){
            src_pc_rearr.block<2,1>(0,j) = src_pc_2D.block<2,1>(0,neigh.src_indices[j]);
            tar_pc_rearr.block<2,1>(0,j) = tar_pc_2D.block<2,1>(0,neigh.tar_indices[j]);
        }

        T = this->getTransform(src_pc_rearr, tar_pc_rearr);

        src_pc_2D = T.block<2,2>(0,0) * src_pc_2D + T.block<2,1>(0,2) * MatrixXd::Ones(1,src_pc_2D.cols());
        Transform_acc = T * Transform_acc;

        mean_dist = std::accumulate(neigh.distances.begin(), neigh.distances.end(), 0.0) / neigh.distances.size();
        if(abs(mean_dist - last_dist) < tolerance){
            // std::cout << "iter times:  " << i+1 << endl;
            break;
        }
        last_dist = mean_dist;
    }
    // std::cout << "cols: " << src_pc_rearr.cols() << endl;
    // std::cout << "mean_dist:  " << mean_dist << endl;

    tar_pc = src_pc;

    this->publishResult(Transform_acc);

    double time_1 = (double)ros::Time::now().toSec();
    // cout<<"time_cost:  "<<time_1-time_0<<endl;
}

void icp_lm::v_left_subfunc(std_msgs::Float64 v_left_msgs)
{
    // time_now = (double)ros::Time::now().toSec();
    double dt = time_now - time_last;
    v_left = 0.08 * v_left_msgs.data;

    double v = (v_left + v_right)/2;

    robot_dx += v * std::cos(robot_dtheta + (v_right - v_left) / tread * dt / 2) * dt;
    robot_dy += v * std::sin(robot_dtheta + (v_right - v_left) / tread * dt / 2) * dt;
    robot_dtheta += (v_right - v_left) / tread * dt;

    time_last = time_now;
    
    // std::cout << "vleft: " << v_left << endl;
    // std::cout << "vright: " << v_right << endl;
    // std::cout << "dt: " << dt << endl;
    // std::cout << "sta: " << robot_dx << robot_dy << robot_dtheta << endl << endl;
}

void icp_lm::v_right_subfunc(std_msgs::Float64 v_right_msgs)
{
    // time_now = (double)ros::Time::now().toSec();
    double dt = time_now - time_last;
    v_right = 0.08 * v_right_msgs.data;

    double v = (v_left + v_right)/2;

    robot_dx += v * std::cos(robot_dtheta + (v_right - v_left) / tread * dt / 2) * dt;
    robot_dy += v * std::sin(robot_dtheta + (v_right - v_left) / tread * dt / 2) * dt;
    robot_dtheta += (v_right - v_left) / tread * dt;

    time_last = time_now;
    // std::cout << "vright: " << v_right << endl;
    // std::cout << "vleft: " << v_left << endl;
    // std::cout << "dt: " << dt << endl;
    // std::cout << "sta: " << robot_dx << robot_dy << robot_dtheta << endl << endl;
}

void icp_lm::clock_subfunc(rosgraph_msgs::Clock clock_msgs)
{
    time_now = clock_msgs.clock.toSec();
    if(isFirstClock){
        isFirstClock = false;
        time_last = time_now;
    }
}

Eigen::MatrixXd icp_lm::landMarksToMatrix(visualization_msgs::MarkerArray input)
{
    int markerSize = input.markers.size();
    // cout<<markerSize<<" markers received !"<<endl;

    Eigen::MatrixXd pc = Eigen::MatrixXd::Ones(3, markerSize);

    for(int i=0; i<markerSize; i++)
    {
        pc(0,i) = input.markers[i].pose.position.x;
        pc(1,i) = input.markers[i].pose.position.y;
    }
    return pc;
}

NeighBor icp_lm::findNearest(const Eigen::MatrixXd &src, const Eigen::MatrixXd &tar)
{
    // TODO: please code by yourself
    double dist;
    double min_dist;
    int min_index;
    Vector2d src_vec;
    Vector2d tar_vec;
    NeighBor neigh;
    // bool isfind[tar.cols()] = {0};
    //std::cout << "here1" << endl;

    for(int i=0; i<src.cols(); i++){
        src_vec = src.block<2,1>(0,i);
        min_index = 0;
        tar_vec = tar.block<2,1>(0,min_index);
        min_dist = this->calc_dist(src_vec, tar_vec);
        for(int j=0; j<tar.cols(); j++){
            // if(isfind[j]){
            //     continue;
            // }
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
            // isfind[min_index] = true;
        }
        //std::cout << "min_dist: " << min_dist << endl;
    }

    //std::cout << "here2" << endl;
    return neigh;
}

Eigen::Matrix3d icp_lm::getWheelTransform(void)
{
    Vector3d sta = Vector3d::Zero(3,1);
    sta <<  robot_dx, 
            robot_dy,   
            robot_dtheta;
    Matrix3d T;

    T = this->staToMatrix(sta);

    robot_dx = 0;
    robot_dy = 0;
    robot_dtheta = 0;

    return T;
}

Eigen::Matrix3d icp_lm::getTransform(const Eigen::MatrixXd &src, const Eigen::MatrixXd &tar)
{
    // TODO: please code by yourself
    Matrix3d T = Matrix3d::Identity(3,3);

    int cols = src.cols();
    //std::cout << "cols: " << cols << endl;
    Vector2d src_mc = Vector2d::Zero();
    Vector2d tar_mc = Vector2d::Zero();
    VectorXd avg = VectorXd::Ones(cols,1) / cols;
    MatrixXd src_q = src;
    MatrixXd tar_q = tar;

    // for(int i=0; i<cols; i++){
    //     src_mc = src_mc + src.block<2,1>(0,i);
    //     tar_mc = tar_mc + tar.block<2,1>(0,i);
    // }
    // src_mc = src_mc / cols;
    // tar_mc = tar_mc / cols;
    // std::cout << "src: " << endl;
    // std::cout << src << endl;
    // std::cout << "tar: " << endl;
    // std::cout << tar << endl;

    src_mc = src * avg;
    tar_mc = tar * avg;

    for(int i=0; i<cols; i++){
        src_q.block<2,1>(0,i) = src.block<2,1>(0,i) - src_mc;
        tar_q.block<2,1>(0,i) = tar.block<2,1>(0,i) - tar_mc;
    }

    // std::cout << "src_q: " << endl;
    // std::cout << src_q << endl;
    // std::cout << "tar_q: " << endl;
    // std::cout << tar_q << endl;

    MatrixXd W = src_q * tar_q.transpose();
    MatrixXd U;
    MatrixXd V;
    Matrix2d R;
    Vector2d t;

    //std::cout << "here1" << endl;
    // std::cout << "W: " << endl;
    // std::cout << W << endl;
    JacobiSVD<MatrixXd> svd(W, ComputeFullU | ComputeFullV);
    //std::cout << "here2" << endl;
    U = svd.matrixU();
    V = svd.matrixV();
    //std::cout << "here3" << endl;
    // std::cout << "U: " << endl;
    // std::cout << U << endl;
    // std::cout << "V: " << endl;
    // std::cout << V << endl;
    R = V * U.transpose();
    // std::cout << "R: " << endl;
    // std::cout << R << endl;
    //std::cout << "here4" << endl;
    if(R.determinant() < 0){
       std::cout << "determinant<0" << endl;
    }

    t = tar_mc - R * src_mc;
    // t = R.inverse() * t;
    //std::cout << "here5" << endl;

    T.block<2,2>(0,0) = R;
    T.block<2,1>(0,2) = t;
    //std::cout << "here6" << endl;
    // std::cout << "T: " << endl;
    // std::cout << T << endl << endl;

    return T;
}

float icp_lm::calc_dist(const Eigen::Vector2d &pta, const Eigen::Vector2d &ptb)
{
    // TODO: please code by yourself
    float dist;
    dist = (pta-ptb).norm();
    // dist = sqrt(pow(pta(0)-ptb(0),2)+pow(pta(1)-ptb(1),2));
    //std::cout << "dist: " << dist << endl;
    return dist;
}

Eigen::Matrix3d icp_lm::staToMatrix(Eigen::Vector3d sta)
{
	Matrix3d RT;
    RT << cos(sta(2)), -sin(sta(2)), sta(0),
          sin(sta(2)), cos(sta(2)),sta(1),
          0, 0, 1;
    return RT;
}

void icp_lm::publishResult(Eigen::Matrix3d T)
{	
    float delta_yaw = atan2(T(1,0), T(0,0));
    // cout<<"sensor-delta-xyt: "<<T(0,2)<<" "<<T(1,2)<<" "<<delta_yaw<<endl;

    sensor_sta(0) = sensor_sta(0) + cos(sensor_sta(2))*T(0,2) - sin(sensor_sta(2))*T(1,2);
    sensor_sta(1) = sensor_sta(1) + sin(sensor_sta(2))*T(0,2) + cos(sensor_sta(2))*T(1,2);
    sensor_sta(2) = sensor_sta(2) + delta_yaw;

    // cout<<"sensor-global: "<<sensor_sta.transpose()<<endl;

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

void publisher(void){
    tf::TransformBroadcaster br;

    tf::Quaternion quat;
    quat.setRPY(0,0,0);

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    transform.setRotation(quat);

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"world_base","map"));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "icp_landMark");
    ros::NodeHandle n;

    publisher();
    icp_lm icp_lm_(n);

    ros::MultiThreadedSpinner spinner(1);
    spinner.spin();

    return 0;
}