/*
    速度控制
    给定机械臂末端的最大速度[0.01414, 0, 0.01414, 0, 0, 0],
    线速度标量为0.02m/s，加速时的加速度标量为 a1 = 0.005m/s2,
    保持最大速度匀速运动3s，减速时的加速度标量为 a2 = 0.004m/s2，
    即最大，加速时间 4s，减速运动时间为 5s，
    使机械臂从起始门位置[0.2289, 0, 0.454, 1.57, 0, 0]开始，完成一段类梯形速度曲线的变速直线运动。
*/
// /probot_anno/arm_vel_controller/command
// /probot_anno/joint_state

#include <string>
#include <vector>
#include <iostream>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <time.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

std::vector<double> joint_angle;

void joint_callback(const sensor_msgs::JointState::ConstPtr& joint_state)
{
    joint_angle = joint_state->position;
    // for(int i=0;i<6;i++)
    //     std::cout << joint_state->position.at(i) << std::endl;
}

double vel_at_time(double time){
    double vel = 0;
    // while(time > 24){
    //     time -= 24;
    // }
    if(time<0){
        vel = 0;
    }else if(time<4){
        vel = time*0.005;
        // vel = 0.1;
    }else if(time<7){
        vel = 0.02;
        // vel = 0.1;
    }else if(time<12){
        vel = (12-time)*0.004;
        // vel = 0.1;
    }
    // else if(time<17){
    //     vel = (12-time)*0.004;
    // }else if(time<20){
    //     vel = -0.02;
    // }else if(time<24){
    //     vel = (time-24)*0.005;
    // }
    return vel;
}

std_msgs::Float64MultiArray rot_2_msg(Eigen::MatrixXd vec){
    std_msgs::Float64MultiArray init_pos;
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    // sleep(1);

    for(int i=0; i<6; i++){
        init_pos.data.at(i) = vec(i,0);
    }
    
    return init_pos;
}

int main(int argc, char** argv)
{
    bool ret;

    ros::init(argc, argv, "experiment_2_vel");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    //定义publisher和subscriber
    ros::NodeHandle node_handle;
    ROS_INFO_STREAM("start");
    ros::Publisher vel_pub = node_handle.advertise<std_msgs::Float64MultiArray>(
                             "/probot_anno/arm_vel_controller/command", 100);
    ros::Subscriber joint_sub = node_handle.subscribe(
                             "/probot_anno/joint_states", 100, joint_callback);
    
    //加载机械臂模型，便于后续计算雅可比矩阵
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    const robot_state::JointModelGroup* joint_model_group = 
                                kinematic_model->getJointModelGroup("manipulator");
    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

    //初始化模型角度
    joint_angle.push_back(0.0);
    joint_angle.push_back(0.0);
    joint_angle.push_back(0.0);
    joint_angle.push_back(0.0);
    joint_angle.push_back(0.0);
    joint_angle.push_back(0.0);
    kinematic_state->setJointGroupPositions(joint_model_group,joint_angle);

    //输出模型角度
    std::vector<double> joint_values;
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }

    //输出尖端Pose
    const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("link_6");
    ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
    ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");

    //输出当前模型状态雅可比矩阵
    Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
    Eigen::MatrixXd jacobian;
    kinematic_state->getJacobian(joint_model_group,
                                kinematic_state->getLinkModel(
                                                joint_model_group->getLinkModelNames().back()),
                                reference_point_position, jacobian);
    ROS_INFO_STREAM("Jacobian: \n" << jacobian << "\n");

    double vel = 0.0;
    double intval = 0.01;
    double past_time = 0;
    ros::Rate loop_rate(1/intval);

    //计算关节角速度
    Eigen::VectorXd vel_hat(6);
    vel_hat(0,0) = 1.0;
    vel_hat(1,0) = 0.0;
    vel_hat(2,0) = -1.0;
    vel_hat(3,0) = 0.0;
    vel_hat(4,0) = 0.0;
    vel_hat(5,0) = 0.0;
    vel_hat = vel_hat.normalized();

    Eigen::VectorXd rot_vec(6);
    rot_vec = jacobian.inverse() * vel_hat * vel;

    ROS_INFO_STREAM("rot_vec: \n" << rot_vec << "\n");

    std_msgs::Float64MultiArray vel_tar_msg = rot_2_msg(rot_vec);
    vel_pub.publish(vel_tar_msg);

    sleep(2);

    ros::spinOnce();
    loop_rate.sleep();
    kinematic_state->setJointGroupPositions(joint_model_group,joint_angle);
    ret = kinematic_state->getJacobian(joint_model_group,
                        kinematic_state->getLinkModel(
                                        joint_model_group->getLinkModelNames().back()),
                        reference_point_position, jacobian);
    if(!ret){
        rot_vec << 0,0,0,0,0,0;
        std_msgs::Float64MultiArray vel_tar_msg = rot_2_msg(rot_vec);
        vel_pub.publish(vel_tar_msg);
        ROS_INFO("No Jacobian!");
    }

    ros::Time begin, end;
    begin = ros::Time::now();
    while(ros::ok()){        
        end = ros::Time::now();
        past_time = end.toSec() - begin.toSec();
        // past_time *= 0.78;
        std::cout << past_time << std::endl;
        vel = vel_at_time(past_time);

        rot_vec = jacobian.inverse() * vel_hat * vel;
        
        std_msgs::Float64MultiArray vel_tar_msg = rot_2_msg(rot_vec);
        vel_pub.publish(vel_tar_msg);

        loop_rate.sleep();
        ros::spinOnce();

        ROS_INFO("\nrot:\t\tjoint:\n");
        for(int i=0; i<6; i++){
            std::cout << rot_vec(i,0) << "\t" << joint_angle.at(i) << std::endl;
        }
        // const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("link_6");
        // ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
        // ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");

        kinematic_state->setJointGroupPositions(joint_model_group,joint_angle);
        ret = kinematic_state->getJacobian(joint_model_group,
                            kinematic_state->getLinkModel(
                                            joint_model_group->getLinkModelNames().back()),
                            reference_point_position, jacobian);
        if(!ret){
            rot_vec << 0,0,0,0,0,0;
            std_msgs::Float64MultiArray vel_tar_msg = rot_2_msg(rot_vec);
            vel_pub.publish(vel_tar_msg);
            ROS_INFO("No Jacobian!");
            break;
        }

        if(past_time > 12.5){
            break;
        }
    }

    ROS_INFO("Program end");
    ros::shutdown();
    return 0;
}