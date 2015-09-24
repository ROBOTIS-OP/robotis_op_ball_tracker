
#include <robotis_op_ball_tracker/robotis_op_ball_tracker_node.h>

using namespace robotis_op;

#include <iostream>

#include <stdlib.h>     /* srand, rand */
#include <std_msgs/Float64.h>
#include <math.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
namespace robotis_op {


RobotisOPBallTrackingNode::RobotisOPBallTrackingNode(ros::NodeHandle nh)
    : nh_(nh)
{


    ros::NodeHandle nh_;

    joint_states_sub_ = nh_.subscribe("/robotis_op/joint_states", 100, &RobotisOPBallTrackingNode::jointStatesCb, this);
    image_sub_ = nh_.subscribe("/robotis_op/camera/image_raw", 100, &RobotisOPBallTrackingNode::imageCb, this);

    tilt_pub_ = nh_.advertise<std_msgs::Float64>("/robotis_op/j_tilt_position_controller/command", 100);
    pan_pub_ = nh_.advertise<std_msgs::Float64>("/robotis_op/j_pan_position_controller/command", 100);
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("robotis_op/cmd_vel", 1);
}

RobotisOPBallTrackingNode::~RobotisOPBallTrackingNode()
{
}


void RobotisOPBallTrackingNode::jointStatesCb(const sensor_msgs::JointState& msg)
{
    pan_ = msg.position.at(10);
    tilt_ = msg.position.at(21);
}

//http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
void RobotisOPBallTrackingNode::imageCb(const sensor_msgs::Image& msg)
{
    ROS_INFO("received image");
     cv_bridge::CvImagePtr image;
    image = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    cv::circle(image->image,cv::Point(50, 50), 10, CV_RGB(255,0,0));
    sensor_msgs::ImagePtr msg_out;
    msg_out = image->toImageMsg();
    ros::Publisher img_pub =nh_.advertise<sensor_msgs::ImagePtr>("/robotis_op/ball_tracker/image_raw", 100);
    img_pub.publish(msg_out);


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

