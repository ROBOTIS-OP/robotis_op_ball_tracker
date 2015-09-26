#ifndef ROBOTIS_OP_BALL_TRACKING_NODE_H
#define ROBOTIS_OP_BALL_TRACKING_NODE_H

#include <ros/ros.h>

#include <robotis_op_ball_tracker/robotis_op_ball_tracker.h>

#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/JointState.h>

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

    void imageCb(const sensor_msgs::Image& msg);
    void jointStatesCb(const sensor_msgs::JointState& msg);
    ros::NodeHandle nh_;

    ros::Subscriber joint_states_sub_;
    ros::Subscriber image_sub_;

    ros::Publisher tilt_pub_;
    ros::Publisher pan_pub_;
    ros::Publisher vel_pub_;
    ros::Publisher transformed_img_pub_;
    ros::Publisher blob_img_pub_;


    double pan_, tilt_;




};

}
#endif //ROBOTIS_OP_BALL_TRACKING_NODE_H
