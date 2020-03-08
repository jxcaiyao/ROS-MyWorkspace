/*
    速度控制
    在仿真实验中，使机械臂末端保持初始门位置坐标[0.2289, 0, 0.454]， 
    在此处定点转动，在真实机器人实验中，可以自选位置.
    但定点转动的幅度不可太小，转动角度须不小于 40°
*/

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

std_msgs::Float64MultiArray vec_2_msg(std::vector<double> vec){
    std_msgs::Float64MultiArray init_pos;
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    // sleep(1);

    for(int i=0; i<6; i++){
        init_pos.data.at(i) = vec[i];
    }
    
    return init_pos;
}

std::vector<double> nearest_sol_rad(
                                    std::vector<std::vector<double>> sol_rad,
                                    std::vector<double> last_sol_rad ){
    std::vector<double> rad_dist;
    for(int i=0; i<sol_rad.size(); i++){
        rad_dist.push_back(0.0);
        for(int j=0; j<6; j++){
            rad_dist[i] += fabs(sol_rad[i][j] - last_sol_rad[j]);
        }
    }

    int min_index = 0;
    for(int i=0; i<rad_dist.size(); i++){
        if(rad_dist[i] < rad_dist[min_index]){
            min_index = i;
        }
    }

    return sol_rad.at(min_index);
}

int main(int argc, char** argv){

    bool ret;

    ros::init(argc,argv,"control_example_copy");

    ros::NodeHandle node_handle;

    ROS_INFO_STREAM("start");

    ros::Publisher pos_pub = node_handle.advertise<std_msgs::Float64MultiArray>("/probot_anno/arm_pos_controller/command", 100);

    IKFastKinematicsPlugin ik;
    
    ret=ik.IKFastKinematicsPlugin::initialize("robot_description","manipulator","base_link","link_6",0.001);

    geometry_msgs::Pose req_pose;
    req_pose.position.x = 0.2289;
    req_pose.position.y = 0;
    req_pose.position.z = 0.454 - 0.022;
    req_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(1.570796,0,0);

    std::vector<geometry_msgs::Pose> req_pose_vec;
    req_pose_vec.push_back(req_pose);

    double rot_vel = 0.8*3.1415927;
    double theta = 0;
    double intval = 0.01;
    ros::Rate loop_rate(1/intval);

    std::vector<double> seed1;
    seed1.push_back(0.0);
    seed1.push_back(0.0);
    seed1.push_back(0.0);
    seed1.push_back(0.0);
    seed1.push_back(0.0);
    seed1.push_back(0.0);

    std::vector<std::vector<double>> sol_rad;
    std::vector<double> last_sol_rad;

    kinematics::KinematicsResult kinematic_result;
    ik.getPositionIK(req_pose_vec, seed1, sol_rad, kinematic_result, kinematics::KinematicsQueryOptions());
    last_sol_rad = sol_rad.at(0);
    while(ros::ok()){

        ret=ik.getPositionIK(req_pose_vec, seed1, sol_rad, kinematic_result, kinematics::KinematicsQueryOptions());

        theta += rot_vel * intval;
        if(theta > 1.4*3.1415927 || theta < 0.4*-3.1415927){
            rot_vel = -rot_vel;
        }
        req_pose_vec.at(0).orientation = tf::createQuaternionMsgFromRollPitchYaw(theta + 1.570796,0,0);

        if(ret){
            std::cout << sol_rad.size() << " IK solved successfully." << endl;
            for(int q=0; q < sol_rad.size(); q++){
                for(int i=0; i<6; i++){
                    cout << sol_rad[q][i] << ",";
                }
                cout << endl;
            }

            last_sol_rad = nearest_sol_rad(sol_rad, last_sol_rad);
            std_msgs::Float64MultiArray init_pos = vec_2_msg(last_sol_rad);

            pos_pub.publish(init_pos);
            ROS_INFO_STREAM("published");
        }else{
            ROS_INFO_STREAM("No Solution!");
        }

        loop_rate.sleep();

        sol_rad.clear();
    }

    return 0;
}