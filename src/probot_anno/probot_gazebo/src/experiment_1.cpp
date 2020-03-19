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

class experiment_1{
private:
    const static int N1 = 3;
    const static int N2 = 3;
    const double req_pose_scalar[N1][6]={
        {   0.2,    0.2,    0.2007, 1.57,   -1.57,  0       },
        {   0.15,   0.2,    0.2007, 0,      0,      0       },
        {   0.3,    0,      0.122,  1.57,   0,      0       },
        // {   0.2289, 0,      0.454,  1.57,   0,      0       },
    };
    const double req_joint_scalar[N2][6]={
        {   0.927, -0.687,  -0.396, 0,      1.083,  0.927   },
        {   0.322, -0.855,  -0.021, 0,      0.877,  0.833   },
        {   -0.322,-0.636,  -0.011, 0,      0.647,  -0.322  },
    };

public:

    std::vector<std::vector<double>> Init_Joint(void);
    std::vector<std::vector<geometry_msgs::Pose>> Init_Pose(void);
};

std::vector<std::vector<double>> experiment_1::Init_Joint(void){
    std::vector<double> req_joint;
    std::vector<std::vector<double>> req_joint_vec;

    for(int i=0; i<N2; i++){
        for(int j=0; j<6; j++){
            req_joint.push_back(req_joint_scalar[i][j]);
        }
        req_joint_vec.push_back(req_joint);
        req_joint.clear();
    }

    return req_joint_vec;
}

std::vector<std::vector<geometry_msgs::Pose>> experiment_1::Init_Pose(void){

    geometry_msgs::Pose req_pose;
    std::vector<geometry_msgs::Pose> req_pose_vec;
    std::vector<std::vector<geometry_msgs::Pose>> req_pose_vec_vec;
    for(int i=0; i<N1; i++){
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

std_msgs::Float64MultiArray Vec2Msg(std::vector<double> vec){
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

double DegLimit(double fnum){
    while(fnum < -3.1415926535){
        fnum += 3.1415926535*2;
    }
    while(fnum > 3.1415926535){
        fnum -= 3.1415926535*2;
    }
    return fnum;
}

int main(int argc, char** argv){

    bool ret;

    ros::init(argc,argv,"control_example_copy");

    ros::NodeHandle node_handle;

    ROS_INFO_STREAM("start");

    ros::Publisher pos_pub = node_handle.advertise<std_msgs::Float64MultiArray>("/probot_anno/arm_pos_controller/command", 100);

    IKFastKinematicsPlugin ik;
    
    ret=ik.IKFastKinematicsPlugin::initialize("robot_description","manipulator","base_link","link_6",0.001);

    experiment_1 exp1;

    std::vector<std::vector<geometry_msgs::Pose>> req_pose_vec = exp1.Init_Pose();

    std::vector<double> seed1;
    seed1.push_back(0.0);
    seed1.push_back(0.0);
    seed1.push_back(0.0);
    seed1.push_back(0.0);
    seed1.push_back(0.0);
    seed1.push_back(0.0);

    std::vector<std::vector<double>> sol_rad;

    kinematics::KinematicsResult kinematic_result;

    std::vector<std::vector<double>> req_joint_vec;
    req_joint_vec = exp1.Init_Joint();

    std::vector<geometry_msgs::Pose> sol_pose;

    std::vector<std::string> link_names;
    link_names.push_back("link_6");

    while(ros::ok()){
        std::cout << std::endl;
        std::cout << " Inverse kinematics" << std::endl;
        for(int i=0; i<req_pose_vec.size() && ros::ok(); i++){

            ret=ik.getPositionIK(req_pose_vec.at(i), seed1, sol_rad, kinematic_result, kinematics::KinematicsQueryOptions());

            if(ret){
                std::cout << sol_rad.size() << " IK solved successfully." << endl;
                for(int q=0; q < sol_rad.size(); q++){
                    for(int i=0; i<6; i++){
                        cout << sol_rad[q][i] << ",";
                    }
                    cout << endl;
                }
            }else{
                ROS_INFO_STREAM("No Solution!");
            }

            for(int i=0; i<sol_rad.size(); i++){
                std_msgs::Float64MultiArray init_pos = Vec2Msg(sol_rad.at(i));

                pos_pub.publish(init_pos);
                ROS_INFO_STREAM("published");

                sleep(10);
            }

            sol_rad.clear();

            // sleep(10);
        }

        std::cout << std::endl;
        std::cout << " Forward kinematics" << std::endl;

        for(int i=0; i<req_joint_vec.size() && ros::ok(); i++){
            std_msgs::Float64MultiArray init_pos = Vec2Msg(req_joint_vec.at(i));
            pos_pub.publish(init_pos);

            ret = ik.getPositionFK(link_names,req_joint_vec.at(i), sol_pose);
            if(ret){
                std::cout << " FK solved successfully." << endl;
                for(int q=0; q < sol_pose.size(); q++){
                    tf::Quaternion quat;
                    tf::quaternionMsgToTF(sol_pose[q].orientation, quat);
                    double roll, pitch, yaw;
                    tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);
                    sol_pose[q].position.z += 0.022;
                    // roll -= 1.570796;
                    roll  = DegLimit(roll-1.570796);
                    pitch = DegLimit(pitch);
                    yaw   = DegLimit(yaw);
                    cout << sol_pose[q].position.x << ",";
                    cout << sol_pose[q].position.y << ",";
                    cout << sol_pose[q].position.z << ",";
                    cout << roll    << ",";
                    cout << pitch   << ",";
                    cout << yaw     << endl;
                }
            }else{
                ROS_INFO_STREAM("FK_Solve no solution!");
            }

            sol_pose.clear();

            sleep(10);
        }
    }

    return 0;
}