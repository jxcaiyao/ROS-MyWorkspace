//
// Created by kjwang on 2020/2/19.
//
//本示例程序供大家学习probot_anno的位置及速度控制如何编程，展示了：
//1.如何使用ikfast求解器帮我们去计算机械臂逆解;
//2.如何创建位置控制数据发布者，并发布位置数据（速度控制也一样的，把名字换换而已），控制gazebo的机械臂动起来。
//

#include <string>
#include <ros/ros.h>
#include <iostream>
#include <ctime>
#include "vector"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "controller_manager_msgs/SwitchController.h"
#include "controller_manager_msgs/ListControllers.h"
#include "ikfast.h"
#include "probot_anno_manipulator_ikfast_moveit_plugin.cpp"
#include <Eigen/Dense>

# define pi 3.141592653589793238462643383279502884L /* pi */

//using namespace ikfast_kinematics_plugin;
int main(int argc, char **argv) {


    //节点初始化
    ros::init(argc, argv, "control_example");
    //创建节点句柄对象
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    ROS_INFO_STREAM("start");
    //创建发布者对象，用于发布位置信息，
    //位置控制的话题名为："/probot_anno/arm_pos_controller/command"
    //速度控制的话题名为："/probot_anno/arm_vel_controller/command"
    //发送的数据类型均为：std_msgs::Float32MultiArray，
    //它的创建与赋值方法在下面有，一组6个浮点数
    ros::Publisher pos2_pub = node_handle.advertise<std_msgs::Float32MultiArray>("position_chatter", 1000);

    //创建用于保存关节角的变量，它是vector矩阵，vector<std::vector<double>>类型，三组给定关节变量装在里面
    //内容六关节角度，每个角度是浮点数，为弧度制

    //给定关节角
    double Specify_joint_angle[6] = {0.927, -0.687, -0.396, 0, 1.083, 0.927};
    //double Specify_joint_angle[6] = {0.322, -0.855, -0.021, 0, 0.877, 0.322};
    //double Specify_joint_angle[6] = {-0.322, -0.636, -0.011, 0, 0.647, -0.322};
    //double Specify_joint_angle[6] = {0, 0, 0, 0, 0, 0};

    //DH参数
    double theta[6] = {Specify_joint_angle[0], (double)(Specify_joint_angle[1]+pi/2), Specify_joint_angle[2], Specify_joint_angle[3], (double)(Specify_joint_angle[4]-pi/2), Specify_joint_angle[5]};
    double alpha[6] = {0, pi/2, 0, pi/2, -pi/2, pi/2};
    double di[6] = {0.262, 0, 0, 0.2289, 0, 0.055};
    double ai[6] = {0, 0, 0.225, 0, 0, 0};

    //状态转移矩阵
    std::vector<Eigen::Matrix4d> R1_6(6);
    Eigen::Matrix4d R;
    for(int i=0; i<6; i++){
        R << cos(theta[i]),               -sin(theta[i]),              0,              ai[i],
             sin(theta[i])*cos(alpha[i]), cos(theta[i])*cos(alpha[i]), -sin(alpha[i]), -sin(alpha[i])*di[i],
             sin(theta[i])*sin(alpha[i]), cos(theta[i])*sin(alpha[i]), cos(alpha[i]),  cos(alpha[i])*di[i],
             0,                           0,                           0,              1;
        R1_6[i] = R;
    }
    Eigen::Matrix4d R_end = R1_6[0]*R1_6[1]*R1_6[2]*R1_6[3]*R1_6[4]*R1_6[5];
    Eigen::Matrix3d R_correct;
    //纠正矩阵，使结果与仿真相同
    R_correct << 1,0,0, 0,0,1, 0,-1,0;
    Eigen::Matrix3d R_posture = R_end.block<3,3>(0,0)*R_correct;
    std::cout << R_posture << std::endl;

    double end_pitch = atan2(-1*R_posture(2,0),sqrt(R_posture(0,0)*R_posture(0,0)+R_posture(1,0)*R_posture(1,0)));
    double end_yaw = atan2(R_posture(1,0)/cos(end_pitch), R_posture(0,0)/cos(end_pitch));
    double end_roll = atan2(R_posture(2,1)/cos(end_pitch), R_posture(2,2)/cos(end_pitch));
    double end_x = R_end(0,3);
    double end_y = R_end(1,3);
    double end_z = R_end(2,3)+0.022;

    std::cout << "Calculation_x: " << end_x << std::endl;
    std::cout << "Calculation_y: " << end_y << std::endl;
    std::cout << "Calculation_z: " << end_z << std::endl;
    std::cout << "Calculation_rool: " << end_roll << std::endl;
    std::cout << "Calculation_pitch: " << end_pitch << std::endl;
    std::cout << "Calculation_yaw: " << end_yaw << std::endl;

    //初始化publish的变量并初始化6个值
    std_msgs::Float32MultiArray init_pos;
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    sleep(1);


    //为要发送的变量装入解出的六关节坐标
    //如果是要发送速度，那也是类似于这样，装入6个关节角速度值（弧度制）即可
    init_pos.data.at(0) = Specify_joint_angle[0] * 30 * 180 / 3.1415926535;
    init_pos.data.at(1) = Specify_joint_angle[1] * 205 * 180 / 3 / 3.1415926535;
    init_pos.data.at(2) = Specify_joint_angle[2] * 50 * 180 / 3.1415926535;
    init_pos.data.at(3) = Specify_joint_angle[3] * 125 * 180 / 2 / 3.141592653;
    init_pos.data.at(4) = Specify_joint_angle[4] * 125 * 180 / 2 / 3.141592653;
    init_pos.data.at(5) = Specify_joint_angle[5] * 200 * 180 / 9 / 3.141592653;
    init_pos.data.at(6) = 1200;

    //发送出去，若成功，机械臂状态会改变
    pos2_pub.publish(init_pos);
    ROS_INFO_STREAM("published");
}
