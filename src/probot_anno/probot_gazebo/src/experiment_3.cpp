/*
    五次多项式插值，速度控制
    给定四个机械臂坐标系中的空间路径点位姿： 
    第一个点：[0.2289, 0, 0.454, 1.57, 0, 0]（初始门位置） 
    第二个点：[0.3, 0.25, 0.322, 1.57, -1.57, 0] 
    第三个点：[0.3, 0.1, 0.172, 1.57, -1.57, 0] 
    第四个点：[0.3, -0.1, 0.122, 1.57, -1.57, 0]
*/

/*
    各点关节角：
    {   0,          0,          0,          0,          0,          0,          },
    {   0.795586,   -0.772868,  0.205274,   1.08736,    2.51044,    -0.842893,  },
    {   0.387677,   -1.07535,   -0.199738,  0.40437,    2.86772,    -0.114908,  },
    {   -0.38737,   -1.29274,   -0.065646,  -0.394642,  2.94427,    0.0811437,  },
*/
/*
    fixed_t = [4,2,2]

    0.000000,	0.000000,	0.000000,	0.048560,	-0.011244,	0.000553,	
    0.795586,	0.160200,	-0.142800,	-0.026561,	-0.001948,	0.002705,	
    0.387677,	-0.575700,	-0.132550,	0.093566,	0.051494,	-0.020809,	

    0.000000,	0.000000,	0.000000,	-0.061361,	0.018745,	-0.001606,	
    -0.772868,	-0.202200,	0.035400,	0.022598,	-0.022573,	0.004397,	
    -1.075350,	-0.160000,	-0.019000,	-0.003237,	0.029553,	-0.008386,	

    0.000000,	0.000000,	0.000000,	0.057087,	-0.021676,	0.002052,	
    0.205274,	-0.182900,	-0.082850,	0.035860,	-0.000376,	0.000354,	
    -0.199738,	-0.067700,	0.151600,	0.041765,	-0.079711,	0.018886,	

    0.000000,	0.000000,	0.000000,	0.168125,	-0.059984,	0.005550,	
    1.087360,	-0.181800,	-0.188900,	0.216463,	-0.126009,	0.022521,	
    0.404370,	-0.570500,	-0.112700,	0.026035,	0.094049,	-0.028759,	

    0.000000,	0.000000,	0.000000,	0.174944,	-0.048613,	0.003671,	
    2.510440,	0.651100,	-0.218150,	-0.136275,	0.099550,	-0.017966,	
    2.867720,	-0.108500,	-0.083900,	0.384288,	-0.243191,	0.045184,	

    0.000000,	0.000000,	0.000000,	-0.142277,	0.050741,	-0.004616,	
    -0.842893,	0.251800,	0.209500,	-0.098719,	0.009314,	0.000847,	
    -0.114908,	0.271000,	-0.091500,	-0.024185,	0.018577,	-0.002615,	
*/

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

const static std::vector<double> fixed_time = {0,4,6,8};
const static double coff[6][3][6] = {
    {  {0.000000,	0.000000,	0.000000,	0.048560,	-0.011244,	0.000553,},	
       {0.795586,	0.160200,	-0.142800,	-0.026561,	-0.001948,	0.002705,},	
       {0.387677,	-0.575700,	-0.132550,	0.093566,	0.051494,	-0.020809,},    },

    {  {0.000000,	0.000000,	0.000000,	-0.061361,	0.018745,	-0.001606,},	
       {-0.772868,	-0.202200,	0.035400,	0.022598,	-0.022573,	0.004397,},	
       {-1.075350,	-0.160000,	-0.019000,	-0.003237,	0.029553,	-0.008386,},    },	

    {  {0.000000,	0.000000,	0.000000,	0.057087,	-0.021676,	0.002052,},	
       {0.205274,	-0.182900,	-0.082850,	0.035860,	-0.000376,	0.000354,},	
       {-0.199738,	-0.067700,	0.151600,	0.041765,	-0.079711,	0.018886,},     },	

    {  {0.000000,	0.000000,	0.000000,	0.168125,	-0.059984,	0.005550,},	
       {1.087360,	-0.181800,	-0.188900,	0.216463,	-0.126009,	0.022521,},	
       {0.404370,	-0.570500,	-0.112700,	0.026035,	0.094049,	-0.028759,},    },	

    {  {0.000000,	0.000000,	0.000000,	0.174944,	-0.048613,	0.003671,},	
       {2.510440,	0.651100,	-0.218150,	-0.136275,	0.099550,	-0.017966,},	
       {2.867720,	-0.108500,	-0.083900,	0.384288,	-0.243191,	0.045184,},     },	

    {  {0.000000,	0.000000,	0.000000,	-0.142277,	0.050741,	-0.004616,},	
       {-0.842893,	0.251800,	0.209500,	-0.098719,	0.009314,	0.000847,},	
       {-0.114908,	0.271000,	-0.091500,	-0.024185,	0.018577,	-0.002615,},    },	
};

void joint_callback(const sensor_msgs::JointState::ConstPtr& joint_state)
{
    joint_angle = joint_state->position;
}

Eigen::VectorXd vel_at_time(double time){
    Eigen::VectorXd vel_vec(6);
    vel_vec = vel_vec.Zero(6);
    int j;   

    if(time < 0 || time > fixed_time.back()){
        return vel_vec.setZero(6);
    }
    j = 0;
    if(time > 4){
        if(time > 6){
            time -= 6;
            j = 2;
        }else{
            time -= 4;
            j = 1;
        }
    }

    Eigen::Matrix<double,6,6> ddt;
    ddt = ddt.Zero();
    for(int i=0; i<5; i++){
        ddt(i,i+1) = i+1;
    }

    Eigen::Matrix<double,1,6> t1; 
    t1 = t1.Zero();   
    for(int i=0; i<6; i++){
        t1(0,i) = pow(time,i);
    }

    Eigen::Matrix<double,6,1> co;
    co = co.Zero();
    
    //计算每个关节速度
    for(int i=0; i<6; i++){
        for(int k=0; k<6; k++){
            co(k,0) = coff[i][j][k];
        }
        vel_vec(i) = t1 * ddt * co;
    }

    return vel_vec;
}

std_msgs::Float64MultiArray rot_2_msg(Eigen::VectorXd vec){
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

    ros::init(argc, argv, "experiment_3");
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

    double vel = 0.0;
    double intval = 0.01;
    double past_time = 0;
    ros::Rate loop_rate(1/intval);

    Eigen::Matrix<double,6,1> rot_vec;
    rot_vec = rot_vec.Zero();

    std_msgs::Float64MultiArray vel_tar_msg = rot_2_msg(rot_vec);

    sleep(2);

    ROS_INFO("begin:\n");
    ros::Time begin, end;
    begin = ros::Time::now();
    while(ros::ok()){        
        end = ros::Time::now();
        past_time = end.toSec() - begin.toSec();
        
        // std::cout << past_time << std::endl;

        rot_vec = vel_at_time(past_time);
        // std::cout << rot_vec << "\n\n";
        vel_tar_msg = rot_2_msg(rot_vec);
        vel_pub.publish(vel_tar_msg);

        loop_rate.sleep();
        ros::spinOnce();

        kinematic_state->setJointGroupPositions(joint_model_group,joint_angle);

        for(int i=0; i<4; i++){
            if(fabs(past_time - fixed_time.at(i)) < 1e-2){
                ROS_INFO_STREAM("Pose at time:" << past_time << "\n");
                const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("link_6");
                ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
                ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");
                break;
            }
        }

        if(past_time > fixed_time.back()){
            break;
        }
    }
    vel_tar_msg = rot_2_msg(rot_vec.Zero());
    vel_pub.publish(vel_tar_msg);

    ROS_INFO("Program end");
    ros::shutdown();
    return 0;
}