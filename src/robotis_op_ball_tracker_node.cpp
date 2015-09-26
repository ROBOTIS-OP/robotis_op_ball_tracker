
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

    transformed_img_pub_ =nh_.advertise<sensor_msgs::Image>("/robotis_op/ball_tracker/image_transformed_raw", 100);
    blob_img_pub_ =nh_.advertise<sensor_msgs::Image>("/robotis_op/ball_tracker/image_blob_raw", 100);
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

    //DETECT BALL
    cv_bridge::CvImagePtr image_trans, image_blob;
    image_trans = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::RGB8);
    image_blob = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::RGB8);

    cv::Mat& mat_blob = image_blob->image;
    cv::Mat& mat_trans = image_trans->image;
    cv::GaussianBlur( mat_trans, mat_trans, cv::Size(9, 9), 2, 2 );

    std::vector<cv::Mat> canales;
    cv::split(mat_trans, canales);
    canales[0] =  2*canales[0]-canales[1]-canales[2];
    canales[1] = canales[2]-canales[2];
    canales[2] = canales[2]-canales[2];
    cv::merge(canales, mat_trans);
    cv::cvtColor(mat_trans,mat_trans, CV_RGB2GRAY);

    std::vector<cv::Vec3f> circles;

    cv::HoughCircles(mat_trans, circles, CV_HOUGH_GRADIENT, 1,  mat_trans.rows/8, 200, 10, 0, 0 );

    ROS_INFO("detected %i circles",(int)circles.size());
    cv::Point ball_position;
    bool ball_detected = false;
    for( size_t i = 0; i < circles.size(); i++ )
    {
        ball_detected = true;
        cv::Point center((circles[i][0]), (circles[i][1]));
        ball_position = center;
        int radius = (circles[i][2]);
        // circle center
        cv::circle( mat_blob, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
        // circle outline
        cv::circle( mat_blob, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
    }

    sensor_msgs::ImagePtr msg_out_trans;
    image_trans->encoding = "mono8";
    msg_out_trans = image_trans->toImageMsg();
    msg_out_trans->header.stamp = ros::Time::now();
    transformed_img_pub_.publish(*msg_out_trans);


    sensor_msgs::ImagePtr msg_out_blob;
    msg_out_blob = image_blob->toImageMsg();
    msg_out_blob->header.stamp = ros::Time::now();
    blob_img_pub_.publish(*msg_out_blob);

    if(ball_detected)
    {
        cv::Point image_center(mat_blob.size().width/2.0,mat_blob.size().height/2.0);
        cv::Point offset = ball_position - image_center ;
        ROS_INFO("ball pos  %i %i",ball_position.x, ball_position.y);
        ROS_INFO("ball pos offset %i %i",offset.x, offset.y);

        //head movement
        double tilt_scale_ = 0.001;
        double pan_scale_ = 0.001;
        std_msgs::Float64 angle_msg;
        angle_msg.data = tilt_-tilt_scale_*offset.y;
        tilt_pub_.publish(angle_msg);
        angle_msg.data = pan_-pan_scale_*offset.x;
        pan_pub_.publish(angle_msg);
        i_no_detection_ = 0;

        //walking
        double a_scale_ = 0.5;
        geometry_msgs::Twist vel;
        vel.angular.z = a_scale_*(angle_msg.data);
        vel.linear.x = std::max(0.0,0.8- std::abs(5.0*vel.angular.z));//l_scale_*joy->axes[axis_linear_x_];
        vel.linear.y = 0;//l_scale_*joy->axes[axis_linear_y_];
        vel_pub_.publish(vel);
    }
    else if (i_no_detection_< 10)
    {
        i_no_detection_++;
    }
    else
    {
        geometry_msgs::Twist vel;
        vel.angular.z = 0;
        vel.linear.x = 0;//l_scale_*joy->axes[axis_linear_x_];
        vel.linear.y = 0;//l_scale_*joy->axes[axis_linear_y_];
        vel_pub_.publish(vel);
        //todo search loop
    }

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

