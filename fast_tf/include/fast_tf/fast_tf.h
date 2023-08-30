#include <ros/ros.h>
#include <math.h>
#include <string>
#include <eigen3/Eigen/Dense>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>

#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_msgs/TFMessage.h>

#include <pose_tf/euler2quaternion.h>

using namespace ros;
using namespace std;
using namespace Eigen;

#ifndef FAST_TF
#define FAST_TF

typedef Matrix<double, 3, 1> Matrix31d;
typedef Matrix<double, 4, 1> Matrix41d;

namespace ftf
{

    class FastTF
    {
    public:
        // Search tf from tf2_msgs/TFMessage (return: found(true) / not found(false))
        static bool TFSearch(string frame_in, string frame_out, tf2_msgs::TFMessage tf_msg);
        static bool TFSearch(string frame_in, string frame_out, tf2_msgs::TFMessage tf_msg, geometry_msgs::TransformStamped& tf_);
        static int TFSearchF(string frame_in, string frame_out, tf2_msgs::TFMessage tf_msg);

        // Search tf from tf2_msgs/TFMessage and transform geometry_msgs/Pose (return: found(true) / not found(false))
        static bool TFPose(geometry_msgs::Pose &pose_, string frame_in, string frame_out, tf2_msgs::TFMessage tf_msg);
        static bool TFPoseF(geometry_msgs::Pose &pose_, string frame_in, string frame_out, geometry_msgs::TransformStamped transform);
        
        // Search tf from tf2_msgs/TFMessage and transform geometry_msgs/PoseStamped (return: found(true) / not found(false))
        static bool TFPoseStamped(geometry_msgs::PoseStamped &pose_, string frame_in, string frame_out, tf2_msgs::TFMessage tf_msg);
        static bool TFPoseStampedF(geometry_msgs::PoseStamped &pose_stamped_, string frame_in, string frame_out, geometry_msgs::TransformStamped transform);

        static geometry_msgs::Pose RelativePose(geometry_msgs::Pose target_pose_, geometry_msgs::Pose reference_pose_);

        // transform geometry_msgs/Pose with geometry_msgs/Transform
        static void TF(geometry_msgs::Pose &pose_, geometry_msgs::Transform tf_, bool reverse = false);
        static void TF(geometry_msgs::Wrench &wrench_, geometry_msgs::Transform tf_, bool reverse = false);
        static void TFOnlyR(geometry_msgs::Pose &pose_, geometry_msgs::Transform tf_, bool reverse = false);
        static void TFOnlyR(geometry_msgs::Wrench &wrench_, geometry_msgs::Transform tf_, bool reverse = false);
        static geometry_msgs::Transform ReverseTF(geometry_msgs::Transform tf_);

        static geometry_msgs::Quaternion Rotate(geometry_msgs::Quaternion orientation_, geometry_msgs::Quaternion rotation_);
        static geometry_msgs::Quaternion Rotate(geometry_msgs::Quaternion orientation_, vector<double> rotation_);
        static geometry_msgs::Quaternion Rotate(geometry_msgs::Quaternion orientation_, geometry_msgs::Vector3 rotation_);

        static void GetAngleDiff2Euler(geometry_msgs::Pose pose1_, geometry_msgs::Pose pose2_, geometry_msgs::Vector3 &angle_diff_);
        static void GetAngleDiff2Quaternion(geometry_msgs::Pose pose1_, geometry_msgs::Pose pose2_, geometry_msgs::Quaternion &angle_diff_);

        static Matrix4d Quaternion2Mat4(geometry_msgs::Quaternion quat_in);
        static Matrix3d Quaternion2Mat3(geometry_msgs::Quaternion quat_in);
        static geometry_msgs::Quaternion Mat42Quaternion(Matrix4d matrix_in);
        static geometry_msgs::Quaternion Mat32Quaternion(Matrix3d matrix_in);
        static geometry_msgs::Vector3 Mat32Euler(Matrix3d matrix_in);
    };
}

#endif