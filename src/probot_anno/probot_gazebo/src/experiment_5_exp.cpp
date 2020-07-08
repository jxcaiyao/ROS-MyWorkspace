/*
    敲铃，五次多项式插值，速度控制
    给定五个机械臂坐标系中的空间路径点位姿，形成循环： 
    第一个点：[0.2289, 0, 0.454, 1.57, 0, 0]
    第二个点：[0.26, 0.15, 0.08, 1.57, 0, 0] 
    第三个点：[0.27, 0.05, 0.30, 1.57, 0, 0] 
    第四个点：[0.28, -0.1, 0.20, 1.57, 0, 0]
    第五个点：[0.28, -0.24,0.08, 1.57, 0, 0]

*/

/*
    各点关节角：
    0,          0,          0,          0,          0,          0
    0.523405,   -1.28363,   0.090545,   -0.001869,  1.19269,    0.525142
    0.183268,   -0.411348,  -0.220813,  -0.000971,  0.632016,   0.183841
    -0.342885,  -0.805852,  -0.134104,  -0.001273,  0.940224,   -0.341857
    -0.708536,  -1.44744,   0.565428,   -0.000952,  0.882532,   -0.707801
*/

#include <string>
#include <vector>
#include <iostream>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <time.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#define PI 3.1415926535897

double factor1 = 0.60;
double factor2 = 2.15;

const static int N_waypoint = 5;
const static double fixed_dt[N_waypoint-1] = {4.5,1.5,1.2,2};
static std::vector<double> fixed_time = {0,0,0,0,0};
const static double coff[6][N_waypoint-1][6] = 
{{	{0.000000,	-0.000000,	0.000000,	0.049756,	-0.016585,	0.001474,	},
	{0.453405,	0.000000,	0.000000,	-0.223289,	0.119719,	-0.019852,	},
	{0.155130,	-0.393500,	-0.058600,	0.092258,	-0.055670,	0.014407,	},
	{-0.321619,	-0.371000,	0.041500,	0.010604,	0.022860,	-0.008172,	},
},
{	{0.000000,	0.000000,	0.000000,	-0.140865,	0.046955,	-0.004174,	},
	{-1.283630,	0.000000,	0.000000,	0.126541,	-0.078756,	0.013122,	},
	{-1.155610,	0.123100,	-0.050900,	-0.097192,	0.060703,	-0.013286,	},
	{-1.156320,	-0.137100,	-0.105900,	0.000600,	0.056400,	-0.015641,	},
},
{	{0.000000,	-0.000000,	0.000000,	0.009936,	-0.003312,	0.000294,	},
	{0.090545,	0.000000,	0.000000,	0.163305,	-0.077905,	0.009284,	},
	{0.317808,	0.285600,	-0.003500,	-0.132216,	0.067499,	-0.010414,	},
	{0.541069,	0.064600,	-0.076250,	0.047924,	-0.015424,	0.001986,	},
},
{	{0.000000,	0.000000,	0.000000,	-0.000205,	0.000068,	-0.000006,	},
	{-0.001869,	0.000000,	0.000000,	0.002569,	-0.002569,	0.000685,	},
	{-0.001002,	-0.000000,	0.000000,	-0.001059,	0.001324,	-0.000441,	},
	{-0.001185,	0.000000,	0.000000,	0.000291,	-0.000218,	0.000044,	},
},
{	{0.000000,	-0.000000,	0.000000,	0.130885,	-0.043628,	0.003878,	},
	{1.192690,	0.000000,	0.000000,	-0.146344,	0.099529,	-0.019090,	},
	{1.057679,	-0.127400,	0.040800,	0.041877,	-0.057665,	0.017125,	},
	{0.958953,	-0.069600,	-0.010750,	0.024999,	-0.006018,	0.000065,	},
},
{	{0.000000,	-0.000000,	0.000000,	0.057629,	-0.019210,	0.001708,	},
	{0.525142,	0.000000,	0.000000,	-0.367559,	0.249247,	-0.051450,	},
	{0.155751,	-0.418500,	-0.025600,	0.035979,	0.026021,	-0.017491,	},
	{-0.310707,	-0.326000,	0.026500,	-0.047118,	0.066151,	-0.016643,	},
},
};

Eigen::VectorXd vel_at_time(double time){
    Eigen::VectorXd vel_vec(6);
    vel_vec = vel_vec.Zero(6);

    double ot = time;

    int j;   
    bool flag = false;

    if(time < 0){
        return vel_vec.setZero(6);
    }else if(time < fixed_dt[0] / factor1){
        time = time * factor1;
        j = 0;
    }else{
        time = time - fixed_dt[0] / factor1;
        time = time * factor2;
        
        double T = fixed_time.at(N_waypoint-1) - fixed_dt[0];
        // std::cout << "T: " << T << "\ttime: " << time << std::endl;
        while(time > T*2){
            time -= T*2;
        }
        if(time < T){
            flag = false;
        }else{
            flag = true;
            time = 2*T - time;
        }
        for(j=1; j<N_waypoint-1; j++){
            if(time>=fixed_time[j]-fixed_dt[0] && time<fixed_time[j+1]-fixed_dt[0]){
                time -= fixed_time[j]-fixed_dt[0];
                break;
            }
        }
    }

    Eigen::Matrix<double,6,6> ddt;
    ddt = ddt.Zero();
    for(int i=0; i<5; i++){
        ddt(i,i+1) = i+1;
    }

    Eigen::Matrix<double,1,6> t1,t2; 
    t1 = t1.Zero();   
    t2 = t2.Zero();
    for(int i=0; i<6; i++){
        t1(0,i) = pow(time,i);
        // t2(0,i) = pow(time+0.06, i);
        if(j==0){
            t2(0,i) = pow(time+0.06*factor1, i);
        }else{
            t2(0,i) = pow(time+0.06*factor2, i);
        }
    }

    Eigen::Matrix<double,6,1> co;
    co = co.Zero();
    
    //计算每个关节速度
    for(int i=0; i<6; i++){
        for(int k=0; k<6; k++){
            co(k,0) = coff[i][j][k];
        }
        // angs_vec(i) = t1 * co;
        vel_vec(i) = (t1+t2)/2 * ddt * co;
        // vel_vec(i) = ((t2 * co) - angs_vec(i))/0.05;
    }

    if(flag){
        vel_vec = -vel_vec;
    }
    std::cout << "j:" << j << "\tflag:" << flag << "\ttime:" << time << "\tot:" << ot <<std::endl;

    if(j==0){
        vel_vec = vel_vec * factor1;
    }else{
        vel_vec = vel_vec * factor2;
    }
    return vel_vec;
}

std_msgs::Float32MultiArray rot_2_msg(Eigen::VectorXd vec){
    std_msgs::Float32MultiArray init_pos;
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);
    init_pos.data.push_back(0);

    init_pos.data.at(0) = vec(0) * 30 * 180 / PI;
    init_pos.data.at(1) = vec(1) * 205 * 180 / 3 / PI;
    init_pos.data.at(2) = vec(2) * 50 * 180 / PI;
    init_pos.data.at(3) = vec(3) * 125 * 180 / 2 / PI;
    init_pos.data.at(4) = vec(4) * 125 * 180 / 2 / PI;
    init_pos.data.at(5) = vec(5) * 200 * 180 / 9 / PI;
    
    return init_pos;
}

int main(int argc, char** argv)
{
    bool ret;
    for(int i=0; i<N_waypoint-1; i++){
        fixed_time[i+1] = fixed_time[i] + fixed_dt[i];
    }

    ros::init(argc, argv, "experiment_5");

    //定义publisher和subscriber
    ros::NodeHandle node_handle;
    ROS_INFO_STREAM("start");
    ros::Publisher vel_pub = node_handle.advertise<std_msgs::Float32MultiArray>(
                             "speed_chatter", 100);

    std::cout << "waiting for 5s" << std::endl;
    sleep(5);
    std::cout << "start!" << std::endl;

    double vel = 0.0;
    double intval = 0.05;
    double past_time = 0;
    ros::Rate loop_rate(1/intval);

    Eigen::Matrix<double,6,1> rot_vec;
    rot_vec = rot_vec.Zero();

    std_msgs::Float32MultiArray vel_tar_msg = rot_2_msg(rot_vec);
    vel_pub.publish(vel_tar_msg);

    ROS_INFO("begin:\n");
    
    double past_timet = 0;
    struct timeval begint, endt;
    gettimeofday(&begint, NULL);
    double dt = 0;
    while(ros::ok()){        
        gettimeofday(&endt, NULL);
        past_timet = (double)((endt.tv_sec*1000.0 + endt.tv_usec/1000.0) - (begint.tv_sec*1000.0 + begint.tv_usec/1000.0))/1000.0;
        
        // std::cout << past_timet << std::endl;

        // 有时候出bug，碰到时间跳变约0.6s，需进行大致纠正
        if(past_timet - past_time > 0.5){
            // std::cout << "wrong!!!" << std::endl;
            // break;
            dt = dt + (past_timet - past_time) - 0.062;
        }
        past_time = past_timet;
        past_timet = past_timet - dt;

        rot_vec = vel_at_time(past_timet);
        // std::cout << rot_vec << "\n\n";
        vel_tar_msg = rot_2_msg(rot_vec);
        vel_pub.publish(vel_tar_msg);

        loop_rate.sleep();
    }
    vel_tar_msg = rot_2_msg(rot_vec.Zero());
    vel_pub.publish(vel_tar_msg);

    ROS_INFO("Program end");
    ros::shutdown();
    return 0;
}