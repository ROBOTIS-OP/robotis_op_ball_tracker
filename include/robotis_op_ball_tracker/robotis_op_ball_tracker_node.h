#ifndef ROBOTIS_OP_BALL_TRACKING_NODE_H
#define ROBOTIS_OP_BALL_TRACKING_NODE_H

#include <ros/ros.h>

#include <robotis_op_ball_tracker/robotis_op_ball_tracker.h>

#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
//#include <robotis_op_simulation_walking/robotis_op_walkingConfig.h>

namespace robotis_op {

class RobotisOPBallTrackingNode {
public:
    RobotisOPBallTrackingNode(ros::NodeHandle nh);
    ~RobotisOPBallTrackingNode();

    void Process();
protected:


private:


    ros::NodeHandle nh_;
    ros::Subscriber cmd_vel_subscriber_;
    ros::Subscriber enable_walking_subscriber_;
    ros::Subscriber imu_subscriber_;

    ros::Publisher j_pelvis_l_publisher_;
    ros::Publisher j_thigh1_l_publisher_;
    ros::Publisher j_thigh2_l_publisher_;
    ros::Publisher j_tibia_l_publisher_;
    ros::Publisher j_ankle1_l_publisher_;
    ros::Publisher j_ankle2_l_publisher_;
    ros::Publisher j_shoulder_l_publisher_;

    ros::Publisher j_pelvis_r_publisher_;
    ros::Publisher j_thigh1_r_publisher_;
    ros::Publisher j_thigh2_r_publisher_;
    ros::Publisher j_tibia_r_publisher_;
    ros::Publisher j_ankle1_r_publisher_;
    ros::Publisher j_ankle2_r_publisher_;
    ros::Publisher j_shoulder_r_publisher_;



};

}
#endif //ROBOTIS_OP_BALL_TRACKING_NODE_H
