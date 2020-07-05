#include <string>
#include <vector>
#include <iostream>
#include <time.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
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

        // {0.3,0.25,0.322,1.57,-1.57,0},
        // {0.3,0.1 ,0.172,1.57,-1.57,0},
        // {0.3,-0.1,0.122,1.57,-1.57,0},

        // {0.26,0.15,0.08,1.57,0,0},
        // {0.32,0.05,0.26,1.57,0,0},
        // {0.30,-0.1,0.20,1.57,0,0},
        // {0.28,-0.24,0.08,1.57,0,0}
    };
    const double req_joint_scalar[N2][6]={
        {   0.927, -0.687,  -0.396, 0,      1.083,  0.927   },
        {   0.322, -0.855,  -0.021, 0,      0.877,  0.833   },
        {   -0.322,-0.636,  -0.011, 0,      0.647,  -0.322  },

        // {   0,          0,          0,          0,          0,          0,          },
        // {   0.795586,   -0.772868,  0.205274,   1.08736,    2.51044,    -0.842893,  },
        // {   0.387677,   -1.07535,   -0.199738,  0.40437,    2.86772,    -0.114908,  },
        // {   -0.38737,   -1.29274,   -0.065646,  -0.394642,  2.94427,    0.0811437,  },

        // {   0.523405,   -1.28363,   0.090545,   -0.001869,  1.19269,    0.525142    },
        // {   0.130941,   -1.08029,   0.449436,   -0.000978,  0.630754,   0.131518    },
        // {   -0.294113,  -0.943674,  0.163174,   -0.001073,  0.780731,   -0.293358   },
        // {   -0.708536,  -1.44744,   0.565428,   -0.000952,  0.882532,   -0.707801   },
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

std_msgs::Float32MultiArray Vec2Msg(std::vector<double> vec){
    std_msgs::Float32MultiArray init_pos;
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    // sleep(1);

    init_pos.data.at(0) = vec.at(0) * 30 * 180 / 3.1415926;
    init_pos.data.at(1) = vec.at(1) * 205 * 180 / 3 / 3.1415926;
    init_pos.data.at(2) = vec.at(2) * 50 * 180 / 3.1415926;
    init_pos.data.at(3) = vec.at(3) * 125 * 180 / 2 / 3.1415926;
    init_pos.data.at(4) = vec.at(4) * 125 * 180 / 2 / 3.1415926;
    init_pos.data.at(5) = vec.at(5) * 200 * 180 / 9 / 3.1415926;
    init_pos.data.at(6) = 1200.0;
    
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

    ros::Publisher pos_pub = node_handle.advertise<std_msgs::Float32MultiArray>("position_chatter", 1000);

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
        sleep(5);
        std::cout << std::endl;
        std::cout << " Inverse kinematics" << std::endl;
        for(int i=0; i<req_pose_vec.size() && ros::ok(); i++){

            std::cout << "Trying to solve IK: " << i << std::endl;

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
                std_msgs::Float32MultiArray init_pos = Vec2Msg(sol_rad.at(i));

                pos_pub.publish(init_pos);
                ROS_INFO_STREAM("published");
                std::cout << "IK solution: " << i << std::endl;

                // sleep(10);
                int pnext = 0;
                do{
                    cin >> pnext;
                }while(pnext != 1);
            }

            sol_rad.clear();

            // sleep(10);
        }

        std::cout << std::endl;
        std::cout << " Forward kinematics" << std::endl;

        for(int i=0; i<req_joint_vec.size() && ros::ok(); i++){
            std_msgs::Float32MultiArray init_pos = Vec2Msg(req_joint_vec.at(i));
            pos_pub.publish(init_pos);

            std::cout << "Trying to solve FK: " << i << std::endl;

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

            // sleep(10);
            int pnext = 0;
            do{
                cin >> pnext;
            }while(pnext != 1);
        }
    }

    return 0;
}