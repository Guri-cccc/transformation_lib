#include <pose_tf/euler2quaternion.h>

using namespace ros;
using namespace std;
using namespace conv;

void PoseTransform::Pose2Twist(geometry_msgs::Pose pose_in_, geometry_msgs::Twist& pose_out_){
    pose_out_.linear.x = pose_in_.position.x;
    pose_out_.linear.y = pose_in_.position.y;
    pose_out_.linear.z = pose_in_.position.z;
 
    Quaternion2Euler(pose_in_.orientation.x, pose_in_.orientation.y, pose_in_.orientation.z, pose_in_.orientation.w,
                            pose_out_.angular.x, pose_out_.angular.y, pose_out_.angular.z);
}
void PoseTransform::Twist2Pose(geometry_msgs::Twist pose_in_, geometry_msgs::Pose& pose_out_){
    pose_out_.position.x = pose_in_.linear.x;
    pose_out_.position.y = pose_in_.linear.y;
    pose_out_.position.z = pose_in_.linear.z;
    Euler2Quaternion(pose_in_.angular.x, pose_in_.angular.y, pose_in_.angular.z,
                            pose_out_.orientation.x, pose_out_.orientation.y, pose_out_.orientation.z, pose_out_.orientation.w);
}
void PoseTransform::Quaternion2Euler(double x_, double y_, double z_, double w_,
                            double& roll_, double& pitch_, double& yaw_){
    tf::Quaternion q(x_, y_, z_, w_);
    tf::Matrix3x3 target_m(q);
    target_m.getRPY(roll_, pitch_, yaw_);
}
void PoseTransform::Euler2Quaternion(double roll_, double pitch_, double yaw_,
                            double& x_, double& y_, double& z_, double& w_){
    // geometry_msgs::Quaternion quaterinon = tf::createQuaternionMsgFromRollPitchYaw(roll_, pitch_, yaw_);
    tf2::Quaternion quaterinon;
    quaterinon.setRPY( roll_, pitch_, yaw_);
    x_ = quaterinon.getX();
    y_ = quaterinon.getY();
    z_ = quaterinon.getZ();
    w_ = quaterinon.getW();
    double r = sqrt(pow(x_,2)+pow(y_,2)+pow(z_,2)+pow(w_,2));

    if (r!=0)
    {
        x_ = x_/r;
        y_ = y_/r;
        z_ = z_/r;
        w_ = w_/r;
    }
}
