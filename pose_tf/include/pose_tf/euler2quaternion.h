#include <ros/ros.h>
#include <math.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

using namespace ros;
using namespace std;

#ifndef POSE_TRANS
#define POSE_TRANS

namespace conv
{

    class PoseTransform
    {
    public:
        static void Pose2Twist(geometry_msgs::Pose pose_in_, geometry_msgs::Twist &pose_out_);
        static void Twist2Pose(geometry_msgs::Twist pose_in_, geometry_msgs::Pose &pose_out_);
        static void Quaternion2Euler(double x_, double y_, double z_, double w_,
                                  double &roll_, double &pitch_, double &yaw_);
        static void Euler2Quaternion(double roll_, double pitch_, double yaw_,
                                  double &x_, double &y_, double &z_, double &w_);
    };

}
#endif