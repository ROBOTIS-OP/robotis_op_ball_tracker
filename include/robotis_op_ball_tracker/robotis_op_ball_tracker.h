#ifndef ROBOTIS_OP_BALL_TRACKING_H
#define ROBOTIS_OP_BALL_TRACKING_H
#include <ros/ros.h>

namespace robotis_op {

class RobotisOPBallTracker {
public:
    RobotisOPBallTracker(ros::NodeHandle nh);
    ~RobotisOPBallTracker();

    void update(ros::Time time, ros::Duration period);

protected:


private:

    ros::NodeHandle nh_;

    ros::Publisher j_pelvis_l_publisher_;
    ros::Publisher j_thigh1_l_publisher_;
    ros::Publisher j_thigh2_l_publisher_;
    ros::Publisher j_tibia_l_publisher_;
    ros::Publisher j_ankle1_l_publisher_;
    ros::Publisher j_ankle2_l_publisher_;

    ros::Publisher j_pelvis_r_publisher_;
    ros::Publisher j_thigh1_r_publisher_;
    ros::Publisher j_thigh2_r_publisher_;
    ros::Publisher j_tibia_r_publisher_;
    ros::Publisher j_ankle1_r_publisher_;
    ros::Publisher j_ankle2_r_publisher_;
};

}
#endif //ROBOTIS_OP_BALL_TRACKING_H
