
#include <robotis_op_ball_tracker/robotis_op_ball_tracker_node.h>

using namespace robotis_op;

#include <iostream>

#include <stdlib.h>     /* srand, rand */
#include <std_msgs/Float64.h>
#include <math.h>

namespace robotis_op {


RobotisOPBallTrackingNode::RobotisOPBallTrackingNode(ros::NodeHandle nh)
    : nh_(nh)
{

    j_pelvis_l_publisher_ = nh_.advertise<std_msgs::Float64>("/robotis_op/j_pelvis_l_position_controller/command",1);
    j_thigh1_l_publisher_ = nh_.advertise<std_msgs::Float64>("/robotis_op/j_thigh1_l_position_controller/command",1);
    j_thigh2_l_publisher_ = nh_.advertise<std_msgs::Float64>("/robotis_op/j_thigh2_l_position_controller/command",1);
    j_tibia_l_publisher_ = nh_.advertise<std_msgs::Float64>("/robotis_op/j_tibia_l_position_controller/command",1);
    j_ankle1_l_publisher_ = nh_.advertise<std_msgs::Float64>("/robotis_op/j_ankle1_l_position_controller/command",1);
    j_ankle2_l_publisher_ = nh_.advertise<std_msgs::Float64>("/robotis_op/j_ankle2_l_position_controller/command",1);
    j_shoulder_l_publisher_ = nh_.advertise<std_msgs::Float64>("/robotis_op/j_shoulder_l_position_controller/command",1);

    j_pelvis_r_publisher_ = nh_.advertise<std_msgs::Float64>("/robotis_op/j_pelvis_r_position_controller/command",1);
    j_thigh1_r_publisher_ = nh_.advertise<std_msgs::Float64>("/robotis_op/j_thigh1_r_position_controller/command",1);
    j_thigh2_r_publisher_ = nh_.advertise<std_msgs::Float64>("/robotis_op/j_thigh2_r_position_controller/command",1);
    j_tibia_r_publisher_ = nh_.advertise<std_msgs::Float64>("/robotis_op/j_tibia_r_position_controller/command",1);
    j_ankle1_r_publisher_ = nh_.advertise<std_msgs::Float64>("/robotis_op/j_ankle1_r_position_controller/command",1);
    j_ankle2_r_publisher_ = nh_.advertise<std_msgs::Float64>("/robotis_op/j_ankle2_r_position_controller/command",1);
    j_shoulder_r_publisher_ = nh_.advertise<std_msgs::Float64>("/robotis_op/j_shoulder_l_position_controller/command",1);

    //cmd_vel_subscriber_ = nh_.subscribe("/robotis_op/cmd_vel", 100, &SimulationWalkingNode::cmdVelCb, this);
    //enable_walking_subscriber_ = nh_.subscribe("/robotis_op/enable_walking", 100, &SimulationWalkingNode::enableWalkCb, this);
    //imu_subscriber_ = nh_.subscribe("/robotis_op/imu", 100, &SimulationWalkingNode::imuCb, this);
}

RobotisOPBallTrackingNode::~RobotisOPBallTrackingNode()
{
}





}



int main(int argc, char **argv)
{

    ros::init(argc, argv, ROS_PACKAGE_NAME);

    ros::NodeHandle nh;
    double control_rate;
    nh.param("robotis_op_walking/control_rate", control_rate, 125.0);
    control_rate = 125.0;

    RobotisOPBallTrackingNode gazebo_walking_node(nh);

    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::Time last_time = ros::Time::now();
    ros::Rate rate(control_rate);


    /**
    dynamic_reconfigure::Server<robotis_op_simulation_walking::robotis_op_walkingConfig> srv;
    dynamic_reconfigure::Server<robotis_op_simulation_walking::robotis_op_walkingConfig>::CallbackType cb;
    cb = boost::bind(&SimulationWalkingNode::dynamicReconfigureCb, &gazebo_walking_node, _1, _2);
    srv.setCallback(cb);
**/

    ROS_INFO("Ball tracking enabled");

    while (ros::ok())
    {
        rate.sleep();
        ros::Time current_time = ros::Time::now();
        ros::Duration elapsed_time = current_time - last_time;
        //gazebo_walking_node.Process();
        last_time = current_time;

    }

    return 0;
}

