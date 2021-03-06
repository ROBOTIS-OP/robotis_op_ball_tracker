#ifndef ROBOTIS_OP_BALL_TRACKING_NODE_H
#define ROBOTIS_OP_BALL_TRACKING_NODE_H

#include <ros/ros.h>

#include <robotis_op_ball_tracker/robotis_op_ball_tracker.h>

#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/JointState.h>

#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <robotis_op_ball_tracker/robotis_op_ball_trackerConfig.h>

namespace robotis_op {

class RobotisOPBallTrackingNode {
public:
    RobotisOPBallTrackingNode(ros::NodeHandle nh);
    ~RobotisOPBallTrackingNode();
    void dynamicReconfigureCb(robotis_op_ball_tracker::robotis_op_ball_trackerConfig &config, uint32_t level);

    void Process();
protected:


private:

    inline float squaredDistXY(cv::Vec3f p1, cv::Vec3f p2)
    {
        return ((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1]));
    }

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


    double dp_, minDist_, param1_, param2_, min_radius_, max_radius_;
    double pan_, tilt_;
    int count_no_detection_, count_search_loop_;

    cv::Vec3f previous_position_;



};

}
#endif //ROBOTIS_OP_BALL_TRACKING_NODE_H
