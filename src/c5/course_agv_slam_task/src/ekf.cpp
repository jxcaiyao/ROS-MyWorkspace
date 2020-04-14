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
#include <visualization_msgs/MarkerArray.h>

using namespace std;
using namespace Eigen;

class ekf{

public:
    ekf(ros::NodeHandle &n);
	~ekf();
    ros::NodeHandle& n;

    // robot init states
    double robot_x;
    double robot_y;
    double robot_theta; 
    // match threshold;
    float match_th;
    // bool
    bool isFirstScan;
    // status
    VectorXd status;

    Vector3d status_pre;
    // covariance
    MatrixXd covariance;
    // noise R
    MatrixXd noise_R;
    // noise Q
    MatrixXd noise_Q;
    // landmark num
    int landMark_num;
    // noises
    float noise_motion, noise_measure;
    // count the non-zero elements in status
    int nonZero_cnt;

    Vector2d mu_t;
    bool is_Predict;
    bool is_Update;
    
    // init all 
    void initAll();
    // predict phase
    void predict(nav_msgs::Odometry odom);
    // update phase
    void update(visualization_msgs::MarkerArray input);
    // landMarks to XY matrix
    Eigen::MatrixXd landMarksToXY(visualization_msgs::MarkerArray input);
    // landMarks to r-phi matrix
    Vector2d cartesianToPolar(double x, double y);
    // update feature map
    void updateFeatureMap(Eigen::MatrixXd newFeatures);
    // get motion Jacobian
    MatrixXd getMotionJacobian();
    // get control jacobian
    MatrixXd getControlJacobian();
    // get observation Jacobian
    MatrixXd getObservJacobian(int i);
    // angle normalization
    double angleNorm(double angle);
    // calc 2D distance
    float calc_dist(const Eigen::Vector2d &pta, const Eigen::Vector2d &ptb);
    // find nearest map points
    int findNearestMap(Vector2d point);

    // ros-related subscribers, publishers and broadcasters
    ros::Subscriber landMark_sub;
    ros::Subscriber icpOdom_sub;
    tf::TransformBroadcaster ekf_broadcaster;
    void publishResult();
 	ros::Publisher odom_pub;
};

ekf::~ekf()
{}

ekf::ekf(ros::NodeHandle& n):
    n(n)
{
    // get the params
	n.getParam("/ekf/robot_x", robot_x);
	n.getParam("/ekf/robot_y", robot_y);
	n.getParam("/ekf/robot_theta", robot_theta);

    n.getParam("/ekf/match_th", match_th);
    n.getParam("/ekf/landMark_num", landMark_num);
    n.getParam("/ekf/noise_motion", noise_motion);
    n.getParam("/ekf/noise_measure", noise_measure);

    this->initAll();

    landMark_sub = n.subscribe("/landMarks", 1, &ekf::update, this);
    icpOdom_sub = n.subscribe("/icp_odom", 1, &ekf::predict, this);
    odom_pub = n.advertise<nav_msgs::Odometry>("/ekf_odom", 1);
}

void ekf::predict(nav_msgs::Odometry odom)
{
    if(!is_Update){
        return;
    }
    is_Update = false;

    // TODO: Please complete the predict phase or motion model
    double roll, pitch, yaw;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(odom.pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    double dx, dy, da;
    dx = odom.pose.pose.position.x - status_pre(0);
    dy = odom.pose.pose.position.y - status_pre(1);
    da = yaw - status_pre(2);
    da = this->angleNorm(da);

    mu_t(0) = sqrt(dx*dx + dy*dy);
    mu_t(1) = da;

    status_pre(0) = odom.pose.pose.position.x;
    status_pre(1) = odom.pose.pose.position.y;
    status_pre(2) = this->angleNorm(yaw);


    status(0) += dx;
    status(1) += dy;
    status(2) += da;
    status(2) = this->angleNorm(status(2));

    is_Predict = true;
}

void ekf::update(visualization_msgs::MarkerArray input)
{   
    double time_0 = (double)ros::Time::now().toSec();

    if(!is_Predict){
        return;
    }
    is_Predict = false;

    MatrixXd landMarkFeatures = this->landMarksToXY(input);
    cout<<"-------------New LM Cnt:    "<<landMarkFeatures.cols()<<endl;

    // TODO: Please complete the update phase or observation model

    // initial
    if(isFirstScan)
    {
        this->updateFeatureMap(landMarkFeatures);
        is_Update = true;
        return;
    }

    // Prediction update

    MatrixXd G_e, G_mu;

    G_e  = this->getMotionJacobian();
    G_mu = this->getControlJacobian();

    // covariance = G_e * covariance * G_e.transpose() + G_mu * noise_R * G_mu.transpose();
    covariance.block(0,0,3,3) = G_e * covariance.block(0,0,3,3) * G_e.transpose();
    covariance.block(0,3,3,2*landMark_num) = G_e * covariance.block(0,3,3,2*landMark_num);
    covariance.block(3,0,2*landMark_num,3) = covariance.block(0,3,3,2*landMark_num).transpose();
    covariance = covariance + G_mu * noise_R * G_mu.transpose();

    // Observation update
    int cols = landMarkFeatures.cols();
    int index;

    for(int i=0; i<cols; i++){
        Vector2d z_t;
        z_t = this->cartesianToPolar(landMarkFeatures(0,i), landMarkFeatures(1,i));
        index = this->findNearestMap(landMarkFeatures.col(i));
        if(index >= 0){     //特征已存在
            Vector2d z_est;
            z_est = this->cartesianToPolar(status(3+2*index)-status(0), status(4+2*index)-status(1));
            z_est(1) -= status(2);
            z_est(1) = this->angleNorm(z_est(1));

            MatrixXd H_t = this->getObservJacobian(index);

            MatrixXd K_t;
            MatrixXd co = MatrixXd::Zero(5,5);
            co.block<3,3>(0,0) = covariance.block<3,3>(0,0);
            co.block<3,2>(0,3) = covariance.block<3,2>(0,2*index+3);
            co.block<2,3>(3,0) = covariance.block<2,3>(2*index+3,0);
            co.block<2,2>(3,3) = covariance.block<2,2>(2*index+3,2*index+3);

            MatrixXd ht = MatrixXd::Zero(2,5);
            ht.block<2,3>(0,0) = H_t.block<2,3>(0,0);
            ht.block<2,2>(0,3) = H_t.block<2,2>(0,2*index+3);
            K_t = covariance * H_t.transpose() * (ht * co * ht.transpose() + noise_Q).inverse();

            Vector2d dz = z_t - z_est;
            dz(1) = this->angleNorm(dz(1));
            if(dz(1) > M_PI){
                dz(1) = dz(1) - 2*M_PI;
            }

            status = status + K_t * dz;
            status(2) = this->angleNorm(status(2));
            covariance = covariance - K_t * (H_t * covariance);
        }else{      //特征不存在
            this->updateFeatureMap(landMarkFeatures.col(i));
        }
    }
    this->publishResult();

    is_Update = true;

    cout << "landMark_num: " << landMark_num << endl;
    double time_1 = (double)ros::Time::now().toSec();
    cout<<"time_cost:  "<<time_1-time_0<<endl<<endl;
}

void ekf::initAll()
{   
    // TODO: You can initial here if you need    
    isFirstScan = true;
    is_Predict = false;
    is_Update = true;
    status = Vector3d::Zero();
    status << robot_x, robot_y, robot_theta;
    status_pre = Vector3d::Zero();
    status_pre << robot_x, robot_y, robot_theta;
    mu_t = Vector2d::Zero();

    covariance = Matrix3d::Zero();
    landMark_num = 0;

    double b = 2*(0.1+0.08/6);
    Matrix2d G = MatrixXd::Zero(2,2);
    G << 1/2, 1/2,
        -1/b, 1/b;
    Matrix2d dSlr = MatrixXd::Zero(2,2);
    dSlr << noise_motion*noise_motion, 0,
            0, noise_motion*noise_motion;

    noise_R = G * dSlr * G.transpose();
        
    noise_Q = MatrixXd::Zero(2,2);
    noise_Q << noise_measure*noise_measure, 0,
               0, noise_measure*noise_measure;
}

Eigen::MatrixXd ekf::landMarksToXY(visualization_msgs::MarkerArray input)
{
    int markerSize = input.markers.size();

    Eigen::MatrixXd pc = Eigen::MatrixXd::Ones(3, markerSize);

    for(int i=0; i<markerSize; i++)
    {
        pc(0,i) = input.markers[i].pose.position.x;
        pc(1,i) = input.markers[i].pose.position.y;
    }
    return pc;
}

void ekf::updateFeatureMap(Eigen::MatrixXd newFeatures)
{   
    // TODO:  Please complete this function if you need
    int cols = newFeatures.cols();
    Matrix3d T;
    T << cos(status(2)), -sin(status(2)), status(0),
         sin(status(2)),  cos(status(2)), status(1),
         0,               0,              1        ;
    if(isFirstScan)
    {   
        // initial the map by landmarks in first scan
        
        landMark_num = cols;

        VectorXd tx = MatrixXd::Zero(3+2*landMark_num,1);
        tx.block<3,1>(0,0) = status.block<3,1>(0,0);

        for(int i=0; i<cols; i++){
            tx.block<2,1>(3+2*i,0) = T.block<2,2>(0,0) * newFeatures.col(i) + T.block<2,1>(0,2);
        }
        status = tx;

        MatrixXd tc = MatrixXd::Identity(3+2*landMark_num, 3+2*landMark_num);
        tc = 100 * tc;
        tc.block<3,3>(0,0) = covariance.block<3,3>(0,0);
        covariance = tc;

        isFirstScan = false;
    }
    else
    {   
        VectorXd tx = MatrixXd::Zero(3+2*landMark_num+2*cols, 1);
        tx.block(0,0,status.rows(),1) = status;

        for(int i=0; i<cols; i++){
            tx.block<2,1>(3+2*landMark_num+2*i,0) = T.block<2,2>(0,0) * newFeatures.col(i) + T.block<2,1>(0,2);
        }
        status = tx;

        MatrixXd tc = MatrixXd::Identity(3+2*landMark_num+2*cols, 3+2*landMark_num+2*cols);
        tc.block(covariance.rows(), covariance.cols(), cols, cols) = 1000 * MatrixXd::Identity(cols, cols);
        tc.block(0, 0, covariance.rows(), covariance.cols()) = covariance;
        landMark_num += cols;

        covariance = tc;
    }
}

int ekf::findNearestMap(Vector2d point)
{   
    // TODO: Please complete the NN search
    int min_index = -1;
    double min_dist = 1e10;
    double cur_dist = 0;
    double min_th = 1e10;
    Vector2d landMark;
    Vector2d p = point;
    Matrix3d T;
    T << cos(status(2)), -sin(status(2)), status(0),
         sin(status(2)),  cos(status(2)), status(1),
         0,               0,              1        ;

    point = T.block<2,2>(0,0) * point + T.block<2,1>(0,2);
    for(int i=0; i<landMark_num; i++){
        landMark(0) = status(i*2 + 3);
        landMark(1) = status(i*2 + 4);

        cur_dist = this->calc_dist(landMark, point);
        if(cur_dist < match_th && cur_dist < min_dist){
            min_index = i;
            min_dist = cur_dist;
        }
    }

    return min_index;
}

Eigen::MatrixXd ekf::getMotionJacobian()
{
    // TODO: Please complete the Jocobian Calculation of Motion
    MatrixXd G_e = MatrixXd::Identity(3,3);
    G_e(0,2) = -mu_t(0) * sin(status(2) + mu_t(1)/2);
    G_e(1,2) =  mu_t(0) * cos(status(2) + mu_t(1)/2);

    return G_e;
}

Eigen::MatrixXd ekf::getControlJacobian()
{
    // TODO: Please complete the Jocobian Calculation of Control
    MatrixXd G_mu = MatrixXd::Zero(3,2);
    MatrixXd F = MatrixXd::Zero(3+2*landMark_num,3);
    F.block<3,3>(0,0) = MatrixXd::Identity(3,3);

    G_mu << cos(status(2) + mu_t(1)/2), -mu_t(0)/2*sin(status(2) + mu_t(1)/2),
            sin(status(2) + mu_t(1)/2),  mu_t(0)/2*cos(status(2) + mu_t(1)/2),
            0,                           1                                   ;
    
    G_mu = F * G_mu;

    return G_mu;
}

Eigen::MatrixXd ekf::getObservJacobian(int i)
{
    // TODO: Please complete the Jocobian Calculation of Observation
    MatrixXd H_v = MatrixXd::Zero(2,5);
    MatrixXd F_i = MatrixXd::Zero(5,3+2*landMark_num);
    double dx = status(3+2*i) - status(0);
    double dy = status(4+2*i) - status(1);
    double q = dx*dx + dy*dy;
    double ds = sqrt(q);
    
    H_v <<  -ds*dx, -ds*dy, 0,  ds*dx,  ds*dy,
            dy,     -dx,    -q, -dy,    dx   ;
    H_v /= q;

    F_i.block<3,3>(0,0) = MatrixXd::Identity(3,3);
    F_i.block<2,2>(3,2*i+3) = MatrixXd::Identity(2,2);

    H_v = H_v * F_i;

    return H_v;
}

Vector2d ekf::cartesianToPolar(double x, double y)
{
    float r = std::sqrt(x*x + y*y);
    float phi = angleNorm(std::atan2(y, x));
    Vector2d r_phi(r, phi);
    return r_phi;
}

float ekf::calc_dist(const Eigen::Vector2d &pta, const Eigen::Vector2d &ptb)
{   
    return std::sqrt((pta[0]-ptb[0])*(pta[0]-ptb[0]) + (pta[1]-ptb[1])*(pta[1]-ptb[1]));
}

double ekf::angleNorm(double angle)
{
    // 0 ~ 360
    while(angle > 2*M_PI)
        angle = angle - 2*M_PI;
    while(angle < 0)
        angle = angle + 2*M_PI;
    return angle;
}

void ekf::publishResult()
{
    // tf
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(status(2));

    geometry_msgs::TransformStamped ekf_trans;
    ekf_trans.header.stamp = ros::Time::now();
    ekf_trans.header.frame_id = "world_base";
    ekf_trans.child_frame_id = "ekf_slam";

    ekf_trans.transform.translation.x = status(0);
    ekf_trans.transform.translation.y = status(1);
    ekf_trans.transform.translation.z = 0.0;
    ekf_trans.transform.rotation = odom_quat;

    ekf_broadcaster.sendTransform(ekf_trans);

    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "world_base";

    odom.pose.pose.position.x = status(0);
    odom.pose.pose.position.y = status(1);
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    odom_pub.publish(odom);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ekf");
    ros::NodeHandle n;

    ekf ekf_(n);

    ros::MultiThreadedSpinner spinner(1);
    spinner.spin();

    // ros::spin();

    return 0;
}