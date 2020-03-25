/*
    敲铃，五次多项式插值，速度控制
    给定六个机械臂坐标系中的空间路径点位姿，形成循环： 
    第一个点：[0.2289, 0, 0.454, 1.57, 0, 0]
    第二个点：[0.26, 0.15, 0.08, 1.57, 0, 0] 
    第三个点：[0.27, 0.05, 0.30, 1.57, 0, 0] 
    第四个点：[0.28, -0.1, 0.20, 1.57, 0, 0]
    第五个点：[0.28, -0.24,0.08, 1.57, 0, 0]
    第六个点：[0.2289, 0, 0.454, 1.57, 0, 0]

*/

/*
    各点关节角：
    0,          0,          0,          0,          0,          0
    0.523405,   -1.28363,   0.090545,   -0.001869,  1.19269,    0.525142
    0.183268,   -0.411348,  -0.220813,  -0.000971,  0.632016,   0.183841
    -0.342885,  -0.805852,  -0.134104,  -0.001273,  0.940224,   -0.341857
    -0.708536,  -1.44744,   0.565428,   -0.000952,  0.882532,   -0.707801
    0,          0,          0,          0,          0,          0
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

const static int N_waypoint = 6;
const static double fixed_dt[N_waypoint-1] = {4,2,1.5,2.5,4};
static std::vector<double> fixed_time = {0,0,0,0,0,0};
const static double coff[6][N_waypoint-1][6] = 
{{	{0.000000,	0.000000,	-0.021400,	0.081120,	-0.025856,	0.002239,	},
	{0.523405,	-0.030000,	-0.096850,	0.006631,	0.002608,	-0.000489,	},
	{0.155130,	-0.293500,	-0.033600,	-0.006612,	0.019790,	-0.005105,	},
	{-0.321619,	-0.301000,	0.031500,	0.001097,	0.007699,	-0.001528,	},
	{-0.708536,	0.059900,	0.089750,	-0.004583,	-0.005250,	0.000646,	},
},
{	{0.000000,	0.000000,	-0.130900,	-0.044492,	0.023292,	-0.002251,	},
	{-1.283630,	-0.100700,	0.130900,	0.088675,	-0.069381,	0.011454,	},
	{-0.995610,	0.183100,	-0.085900,	-0.026919,	0.012844,	-0.002141,	},
	{-0.956320,	-0.137100,	-0.105900,	0.000579,	0.022987,	-0.004029,	},
	{-1.447440,	-0.006000,	0.130900,	0.044263,	-0.021153,	0.001877,	},
},
{	{0.000000,	0.000000,	0.043100,	-0.017490,	0.002045,	-0.000003,	},
	{0.090545,	0.024800,	0.027550,	0.023204,	-0.019697,	0.003656,	},
	{0.237808,	0.075600,	-0.013500,	0.027533,	-0.018551,	0.004112,	},
	{0.351069,	0.074600,	-0.001250,	0.006818,	-0.000251,	-0.000625,	},
	{0.565428,	0.058400,	-0.057200,	-0.031642,	0.013079,	-0.001165,	},
},
{	{0.000000,	0.000000,	0.000000,	-0.000292,	0.000110,	-0.000011,	},
	{-0.001869,	0.000000,	0.000000,	0.001084,	-0.000813,	0.000163,	},
	{-0.001002,	0.000000,	0.000000,	-0.000542,	0.000542,	-0.000145,	},
	{-0.001185,	0.000000,	0.000000,	0.000149,	-0.000089,	0.000014,	},
	{-0.000952,	-0.000000,	0.000000,	0.000104,	-0.000035,	0.000003,	},
},
{	{0.000000,	0.000000,	0.106150,	0.056708,	-0.025808,	0.002414,	},
	{1.192690,	0.054100,	-0.146050,	0.029461,	0.011135,	-0.003215,	},
	{1.027679,	-0.077400,	0.040800,	-0.007192,	-0.005271,	0.002177,	},
	{0.968953,	-0.019600,	0.010750,	0.026343,	-0.022525,	0.003724,	},
	{0.882532,	-0.152400,	-0.054500,	0.008229,	0.000345,	0.000008,	},
},
{	{0.000000,	0.000000,	0.000000,	0.065028,	-0.021808,	0.001900,	},
	{0.525142,	-0.028800,	-0.096900,	0.007011,	0.001942,	-0.000355,	},
	{0.155751,	-0.298500,	-0.036600,	0.000250,	0.016409,	-0.003987,	},
	{-0.320707,	-0.286000,	0.051500,	0.016120,	-0.012028,	0.002294,	},
	{-0.707801,	-0.030000,	0.079750,	0.033396,	-0.016710,	0.001646,	},
},
};

void joint_callback(const sensor_msgs::JointState::ConstPtr& joint_state)
{
    joint_angle = joint_state->position;
}

Eigen::VectorXd vel_at_time(double time){
    Eigen::VectorXd vel_vec(6);
    vel_vec = vel_vec.Zero(6);

    double kp = 0.5;
    Eigen::VectorXd angs_vec(6);
    angs_vec = angs_vec.Zero(6);

    Eigen::VectorXd angm_vec(6);
    angm_vec = angm_vec.Zero(6);
    for(int i=0; i<6; i++){
        angm_vec(i) = joint_angle.at(i);
    }

    int j;   
    
    while(time > fixed_time.back()){
        time -= fixed_time.back();
    }

    if(time < 0){
        return vel_vec.setZero(6);
    }
    for(j=0; j<N_waypoint-1; j++){
        if(time>=fixed_time[j] && time<fixed_time[j+1]){
            time -= fixed_time[j];
            break;
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
        angs_vec(i) = t1 * co;
        vel_vec(i) = t1 * ddt * co;
    }

    vel_vec = vel_vec + kp * (angs_vec - angm_vec);

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
    for(int i=0; i<N_waypoint-1; i++){
        fixed_time[i+1] = fixed_time[i] + fixed_dt[i];
    }

    ros::init(argc, argv, "experiment_5");
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

        for(int i=0; i<N_waypoint-1; i++){
            if(fabs(fmod(past_time, fixed_time.back()) - fixed_time.at(i)) < 1e-2){
                ROS_INFO_STREAM("Pose at time:" << past_time << "\n");
                const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("link_6");
                ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
                ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");
                break;
            }
        }

        // if(past_time > fixed_time.back()){
        //     past_time -= fixed_time.back();
        // }
    }
    vel_tar_msg = rot_2_msg(rot_vec.Zero());
    vel_pub.publish(vel_tar_msg);

    ROS_INFO("Program end");
    ros::shutdown();
    return 0;
}