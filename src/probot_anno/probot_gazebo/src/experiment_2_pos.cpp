/*
    速度控制
    给定机械臂末端的最大速度[0.01414, 0, 0.01414, 0, 0, 0],
    线速度标量为0.02m/s，加速时的加速度标量为 a1 = 0.005m/s2,
    保持最大速度匀速运动3s，减速时的加速度标量为 a2 = 0.004m/s2，
    即最大，加速时间 4s，减速运动时间为 5s，
    使机械臂从起始门位置[0.2289, 0, 0.454, 1.57, 0, 0]开始，完成一段类梯形速度曲线的变速直线运动。
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

double vel_at_time(double time){
    double vel = 0;
    while(time > 24){
        time -= 24;
    }
    if(time<0){
        vel = 0;
    }else if(time<4){
        vel = time*0.005;
    }else if(time<7){
        vel = 0.02;
    }else if(time<12){
        vel = (12-time)*0.004;
    }
    else if(time<17){
        vel = (12-time)*0.004;
    }else if(time<20){
        vel = -0.02;
    }else if(time<24){
        vel = (time-24)*0.005;
    }
    return vel;
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
    req_pose.position.z = 0.354 - 0.022;
    req_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(1.57096+1.570796,0,0);

    std::vector<geometry_msgs::Pose> req_pose_vec;
    req_pose_vec.push_back(req_pose);

    double vel = 0;
    double intval = 0.01;
    double past_time = 0;
    ros::Rate loop_rate(1/intval);
    std::vector<double> vel_vec;
    vel_vec.push_back(1/sqrt(2));
    vel_vec.push_back(0);
    vel_vec.push_back(1/sqrt(2));
    vel_vec.push_back(0);
    vel_vec.push_back(0);
    vel_vec.push_back(0);

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
    sleep(2);
    struct timeval begin, end;
    gettimeofday(&begin, NULL);
    while(ros::ok()){
        gettimeofday(&end, NULL);
        intval = 1000000 * (end.tv_sec - begin.tv_sec) + end.tv_usec - begin.tv_usec;
        intval /= 1000000;
        past_time += intval;
        std::cout << past_time << std::endl;
        vel = vel_at_time(past_time);
        req_pose_vec.at(0).position.x += vel*vel_vec.at(0)*intval;
        req_pose_vec.at(0).position.z += vel*vel_vec.at(2)*intval;
        // past_time += intval;
        begin = end;

        ret = ik.getPositionIK(req_pose_vec, seed1, sol_rad, 
                               kinematic_result, kinematics::KinematicsQueryOptions());

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