#include <string>
#include <vector>
#include <iostream>
#include <time.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_datatypes.h>
#include "controller_manager_msgs/SwitchController.h"
#include "controller_manager_msgs/ListControllers.h"
#include "ikfast.h"
#include "probot_anno_manipulator_ikfast_moveit_plugin.cpp"

using namespace ikfast_kinematics_plugin;

std::vector<std::vector<geometry_msgs::Pose>> Init_Pose(void){
    int N = 3;
    double req_pose_scalar[][6]={
        {0.2,   0.2,    0.2007, 1.57,   -1.57,  0},
        {0.15,  0.2,    0.2007, 0,      0,      0},
        {0.3,   0,      0.122,  1.57,   0,      0},
        // {0.2,   0,      0.3,    1.57,   0,      0},
    };

    geometry_msgs::Pose req_pose;
    std::vector<geometry_msgs::Pose> req_pose_vec;
    std::vector<std::vector<geometry_msgs::Pose>> req_pose_vec_vec;
    for(int i=0; i<N; i++){
        req_pose.position.x = req_pose_scalar[i][0];
        req_pose.position.y = req_pose_scalar[i][1];
        req_pose.position.z = req_pose_scalar[i][2] - 0.022;
        req_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(
            req_pose_scalar[i][3] + 1.570796,
            req_pose_scalar[i][4],
            req_pose_scalar[i][5]
        );
        req_pose_vec.push_back(req_pose);
        req_pose_vec_vec.push_back(req_pose_vec);
        req_pose_vec.clear();
    }
    
    return req_pose_vec_vec;
}

int main(int argc, char** argv){

    bool ret;

    ros::init(argc,argv,"control_example_copy");

    ros::NodeHandle node_handle;

    ROS_INFO_STREAM("start");

    ros::Publisher pos_pub = node_handle.advertise<std_msgs::Float64MultiArray>("/probot_anno/arm_pos_controller/command", 100);

    IKFastKinematicsPlugin ik;
    
    ret=ik.IKFastKinematicsPlugin::initialize("robot_description","manipulator","base_link","link_6",0.001);

    // geometry_msgs::Pose target_pose;
    // target_pose.position.x = 0.2;
    // target_pose.position.y = 0.1;
    // target_pose.position.z = 0.2007 - 0.0225;
    // target_pose.orientation=tf::createQuaternionMsgFromRollPitchYaw(0,0,0);

    std::vector<std::vector<geometry_msgs::Pose>> req_pose_vec = Init_Pose();

    std::vector<double> seed1;
    seed1.push_back(0.0);
    seed1.push_back(0.0);
    seed1.push_back(0.0);
    seed1.push_back(0.0);
    seed1.push_back(0.0);
    seed1.push_back(0.0);

    std::vector<std::vector<double>> sol_rad;

    kinematics::KinematicsResult kinematic_result;

    for(int i=0; i<req_pose_vec.size() && ros::ok(); i=(i+1)%req_pose_vec.size()){

        ret=ik.getPositionIK(req_pose_vec.at(i), seed1, sol_rad, kinematic_result, kinematics::KinematicsQueryOptions());

        if(ret){
            std::cout << sol_rad.size() << " IK solved successfully." << endl;
            for(int q=0; q < sol_rad.size(); q++){
                for(int i=0; i<6; i++){
                    cout << sol_rad[q][i] << " ";
                }
                cout << endl;
            }
        }else{
            ROS_INFO_STREAM("No Solution!");
        }

        std_msgs::Float64MultiArray init_pos;
        init_pos.data.push_back(0);
        init_pos.data.push_back(0);
        init_pos.data.push_back(0);
        init_pos.data.push_back(0);
        init_pos.data.push_back(0);
        init_pos.data.push_back(0);
        // sleep(1);

        for(int i=0; i<6; i++){
            init_pos.data.at(i) = sol_rad[0][i];
        }

        pos_pub.publish(init_pos);
        ROS_INFO_STREAM("published");

        sol_rad.clear();

        sleep(10);
    }
}