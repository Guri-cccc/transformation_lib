#include <fast_tf/fast_tf.h>

using namespace ros;
using namespace std;
using namespace Eigen;
using namespace ftf;

bool FastTF::TFSearch(string frame_in, string frame_out, tf2_msgs::TFMessage tf_msg){
    bool tf_found = false;
    geometry_msgs::Transform tf;

    for (auto transform: tf_msg.transforms){
        if (!transform.header.frame_id.compare(frame_in) && !transform.child_frame_id.compare(frame_out)){
            tf_found = true;
        }
        else if (!transform.header.frame_id.compare(frame_out) && !transform.child_frame_id.compare(frame_in)){
            tf_found = true;
        }
    }

    return tf_found;
}

bool FastTF::TFSearch(string frame_in, string frame_out, tf2_msgs::TFMessage tf_msg, geometry_msgs::TransformStamped& tf_){
    bool tf_found = false;
    geometry_msgs::Transform tf;

    for (auto transform: tf_msg.transforms){
        if (!transform.header.frame_id.compare(frame_in) && !transform.child_frame_id.compare(frame_out)){
            tf_found = true;
            tf_ = transform;
        }
        else if (!transform.header.frame_id.compare(frame_out) && !transform.child_frame_id.compare(frame_in)){
            tf_found = true;
            tf_.transform = ReverseTF(transform.transform);
            tf_.header.frame_id = frame_out;
            tf_.child_frame_id = frame_out;
        }
    }

    return tf_found;
}

int FastTF::TFSearchF(string frame_in, string frame_out, tf2_msgs::TFMessage tf_msg){
    bool tf_found = false;
    geometry_msgs::Transform tf;
    
    for (int i=0; i < sizeof(tf_msg.transforms); ++i){
        if (tf_msg.transforms[i].header.frame_id == frame_in && tf_msg.transforms[i].child_frame_id == frame_out){
            return i;
        }
        else if (tf_msg.transforms[i].header.frame_id == frame_out && tf_msg.transforms[i].child_frame_id == frame_in){
            return i;
        }
    }

    return NULL;
}

bool FastTF::TFPose(geometry_msgs::Pose &pose_, string frame_in, string frame_out, tf2_msgs::TFMessage tf_msg){
    bool tf_found = false;
    geometry_msgs::Transform tf;

    for (auto transform: tf_msg.transforms){
        if (!transform.header.frame_id.compare(frame_in) && !transform.child_frame_id.compare(frame_out)){
            tf_found = true;
            TF(pose_, transform.transform, 0);
        }
        else if (!transform.header.frame_id.compare(frame_out) && !transform.child_frame_id.compare(frame_in)){
            tf_found = true;
            TF(pose_, transform.transform, 1);

        }
    }

    return tf_found;
}

bool FastTF::TFPoseF(geometry_msgs::Pose &pose_, string frame_in, string frame_out, geometry_msgs::TransformStamped transform){
    bool tf_found = false;
    geometry_msgs::Transform tf;

    if (!transform.header.frame_id.compare(frame_in) && !transform.child_frame_id.compare(frame_out)){
        tf_found = true;
        TF(pose_, transform.transform, 0);
    }
    else if (!transform.header.frame_id.compare(frame_out) && !transform.child_frame_id.compare(frame_in)){
        tf_found = true;
        TF(pose_, transform.transform, 1);
    }

    return tf_found;
}

bool FastTF::TFPoseStamped(geometry_msgs::PoseStamped &pose_stamped_, string frame_in, string frame_out, tf2_msgs::TFMessage tf_msg){
    bool tf_found = TFPose(pose_stamped_.pose, frame_in, frame_out, tf_msg);
    if (tf_found){
        pose_stamped_.header.frame_id = frame_out;
    }
    return tf_found;
}

bool FastTF::TFPoseStampedF(geometry_msgs::PoseStamped &pose_stamped_, string frame_in, string frame_out, geometry_msgs::TransformStamped transform){
    bool tf_found = TFPoseF(pose_stamped_.pose, frame_in, frame_out, transform);
    if (tf_found){
        pose_stamped_.header.frame_id = frame_out;
    }
    return tf_found;
}

geometry_msgs::Pose FastTF::RelativePose(geometry_msgs::Pose target_pose_, geometry_msgs::Pose reference_pose_)
{
    geometry_msgs::Pose relative_pose;

    geometry_msgs::Transform tf;
    relative_pose.position.x = target_pose_.position.x - reference_pose_.position.x;
    relative_pose.position.y = target_pose_.position.y - reference_pose_.position.y;
    relative_pose.position.z = target_pose_.position.z - reference_pose_.position.z;
    tf.translation.x = 0.0;
    tf.translation.y = 0.0;
    tf.translation.z = 0.0;
    tf.rotation =  reference_pose_.orientation;
    TF(relative_pose, tf, true);

    Matrix3d tar_r_mat = ftf::FastTF::Quaternion2Mat3(target_pose_.orientation);
    Matrix3d ref_r_mat = ftf::FastTF::Quaternion2Mat3(reference_pose_.orientation);

    Matrix3d manipulation_orientation = ref_r_mat.transpose() * tar_r_mat;
    relative_pose.orientation = Mat32Quaternion(manipulation_orientation);

    return relative_pose;
}

void FastTF::TF(geometry_msgs::Pose &pose_, geometry_msgs::Transform tf_, bool reverse){

    geometry_msgs::Transform tf = tf_;
    // transformation
    if (reverse){
        tf = ReverseTF(tf_);
    }
    
    Matrix4d tf_mat_t = Matrix4d::Zero();
    Matrix3d tf_mat_r = Matrix3d::Zero();

    Matrix41d pose_t_in = Matrix41d::Zero();
    Matrix3d pose_r_in = Matrix3d::Zero();

    Matrix41d pose_t_out = Matrix41d::Zero();
    Matrix3d pose_r_out = Matrix3d::Zero();

    // transformation matrix for position(4x4), for orientation (3x3)
    tf_mat_t = Quaternion2Mat4(tf.rotation);
    tf_mat_r = Quaternion2Mat3(tf.rotation);

    tf_mat_t(0,3) = tf.translation.x;
    tf_mat_t(1,3) = tf.translation.y;
    tf_mat_t(2,3) = tf.translation.z;
    tf_mat_t(3,3) = 1.0;

    // position matrix of the pose (4x1)
    pose_t_in(0,0) = pose_.position.x;
    pose_t_in(1,0) = pose_.position.y;
    pose_t_in(2,0) = pose_.position.z;
    pose_t_in(3,0) = 1.0;

    // orientation matrix of the pose (4x1)
    pose_r_in = Quaternion2Mat3(pose_.orientation);

    // transformation
    pose_t_out = tf_mat_t * pose_t_in;
    pose_r_out = tf_mat_r * pose_r_in;

    // pose out
    pose_.position.x = pose_t_out(0,0);
    pose_.position.y = pose_t_out(1,0);
    pose_.position.z = pose_t_out(2,0);

    pose_.orientation = Mat32Quaternion(pose_r_out);
}

void FastTF::TF(geometry_msgs::Wrench &wrench_, geometry_msgs::Transform tf_, bool reverse){

    geometry_msgs::Transform tf = tf_;
    // transformation
    if (reverse){
        tf = ReverseTF(tf_);
    }
    
    Matrix4d tf_mat_t = Matrix4d::Zero();
    Matrix3d tf_mat_r = Matrix3d::Zero();

    Matrix41d wrench_t_in = Matrix41d::Zero();
    Matrix3d wrench_r_in = Matrix3d::Zero();

    Matrix41d wrench_t_out = Matrix41d::Zero();
    Matrix3d wrench_r_out = Matrix3d::Zero();

    // transformation matrix for position(4x4), for orientation (3x3)
    tf_mat_t = Quaternion2Mat4(tf.rotation);
    tf_mat_r = Quaternion2Mat3(tf.rotation);

    tf_mat_t(0,3) = tf.translation.x;
    tf_mat_t(1,3) = tf.translation.y;
    tf_mat_t(2,3) = tf.translation.z;
    tf_mat_t(3,3) = 1.0;

    // position matrix of the wrench (4x1)
    wrench_t_in(0,0) = wrench_.force.x;
    wrench_t_in(1,0) = wrench_.force.y;
    wrench_t_in(2,0) = wrench_.force.z;
    wrench_t_in(3,0) = 1.0;

    // orientation matrix of the wrench (4x1)
    Vector3d torque(wrench_.torque.x, wrench_.torque.y, wrench_.torque.z);
    wrench_r_in = AngleAxisd(torque.norm(), torque.normalized());

    // transformation
    wrench_t_out = tf_mat_t * wrench_t_in;
    wrench_r_out = tf_mat_r * wrench_r_in;

    // wrench out
    wrench_.force.x = wrench_t_out(0,0);
    wrench_.force.y = wrench_t_out(1,0);
    wrench_.force.z = wrench_t_out(2,0);

    wrench_.torque = Mat32Euler(wrench_r_out);
}

void FastTF::TFOnlyR(geometry_msgs::Pose &pose_, geometry_msgs::Transform tf_, bool reverse){

    geometry_msgs::Transform tf = tf_;
    // transformation
    if (reverse){
        tf = ReverseTF(tf_);
    }

    Matrix3d tf_mat_r = Matrix3d::Zero();

    Matrix3d pose_r_in = Matrix3d::Zero();

    Matrix3d pose_r_out = Matrix3d::Zero();

    // transformation matrix for position(4x4), for orientation (3x3)
    tf_mat_r = Quaternion2Mat3(tf.rotation);
    // orientation matrix of the pose (4x1)
    pose_r_in = Quaternion2Mat3(pose_.orientation);

    // transformation
    pose_r_out = tf_mat_r * pose_r_in;

    pose_.orientation = Mat32Quaternion(pose_r_out);
}

void FastTF::TFOnlyR(geometry_msgs::Wrench &wrench_, geometry_msgs::Transform tf_, bool reverse){
    geometry_msgs::Transform tf = tf_;
    // transformation
    if (reverse){
        tf = ReverseTF(tf_);
    }

    Matrix3d tf_mat_r = Matrix3d::Zero();

    Matrix3d wrench_r_in = Matrix3d::Zero();

    Matrix3d wrench_r_out = Matrix3d::Zero();

    // transformation matrix for position(4x4), for orientation (3x3)
    tf_mat_r = Quaternion2Mat3(tf.rotation);
    // orientation matrix of the wrench (4x1)
    Vector3d torque(wrench_.torque.x, wrench_.torque.y, wrench_.torque.z);
    wrench_r_in = AngleAxisd(torque.norm(), torque.normalized());
    // transformation
    wrench_r_out = tf_mat_r * wrench_r_in;

    wrench_.torque = Mat32Euler(wrench_r_out);
}

geometry_msgs::Transform FastTF::ReverseTF(geometry_msgs::Transform tf_)
{
    Matrix31d tf_mat_t = Matrix31d::Zero();
    tf_mat_t(0, 0) = tf_.translation.x;
    tf_mat_t(1, 0) = tf_.translation.y;
    tf_mat_t(2, 0) = tf_.translation.z;
    Matrix3d tf_mat_r = Matrix3d::Zero();
    tf_mat_r = Quaternion2Mat3(tf_.rotation);

    Matrix3d tf_mat_r_reverse = tf_mat_r.transpose();
    Matrix31d tf_mat_t_reverse = (-1) * tf_mat_r_reverse * tf_mat_t;

    geometry_msgs::Transform reverse_tf;
    reverse_tf.translation.x = tf_mat_t_reverse(0, 0);
    reverse_tf.translation.y = tf_mat_t_reverse(1, 0);
    reverse_tf.translation.z = tf_mat_t_reverse(2, 0);
    reverse_tf.rotation = Mat32Quaternion(tf_mat_r_reverse);
    
    return reverse_tf;
}

geometry_msgs::Quaternion FastTF::Rotate(geometry_msgs::Quaternion orientation_, geometry_msgs::Quaternion rotation_)
{

    Matrix3d tf_mat_r = Matrix3d::Zero();
    Matrix3d pose_r_in = Matrix3d::Zero();
    Matrix3d pose_r_out = Matrix3d::Zero();

    // transformation matrix for position(4x4), for orientation (3x3)
    tf_mat_r = Quaternion2Mat3(rotation_);
    // orientation matrix of the pose (4x1)
    pose_r_in = Quaternion2Mat3(orientation_);

    // transformation
    pose_r_out = tf_mat_r * pose_r_in;

    return Mat32Quaternion(pose_r_out);
}

geometry_msgs::Quaternion FastTF::Rotate(geometry_msgs::Quaternion orientation_, vector<double> rotation_)
{
    geometry_msgs::Quaternion q_rotation;
    conv::PoseTransform::Quaternion2Euler(rotation_[0], rotation_[1], rotation_[2],
                        q_rotation.x, q_rotation.y, q_rotation.z, q_rotation.w);
    Matrix3d tf_mat_r = Matrix3d::Zero();
    Matrix3d pose_r_in = Matrix3d::Zero();
    Matrix3d pose_r_out = Matrix3d::Zero();

    // transformation matrix for position(4x4), for orientation (3x3)
    tf_mat_r = Quaternion2Mat3(q_rotation);
    // orientation matrix of the pose (4x1)
    pose_r_in = Quaternion2Mat3(orientation_);

    // transformation
    pose_r_out = tf_mat_r * pose_r_in;

    return Mat32Quaternion(pose_r_out);
}

geometry_msgs::Quaternion FastTF::Rotate(geometry_msgs::Quaternion orientation_, geometry_msgs::Vector3 rotation_)
{
    geometry_msgs::Quaternion q_rotation;
    conv::PoseTransform::Quaternion2Euler(rotation_.x, rotation_.y, rotation_.z,
                        q_rotation.x, q_rotation.y, q_rotation.z, q_rotation.w);
    Matrix3d tf_mat_r = Matrix3d::Zero();
    Matrix3d pose_r_in = Matrix3d::Zero();
    Matrix3d pose_r_out = Matrix3d::Zero();

    // transformation matrix for position(4x4), for orientation (3x3)
    tf_mat_r = Quaternion2Mat3(q_rotation);
    // orientation matrix of the pose (4x1)
    pose_r_in = Quaternion2Mat3(orientation_);

    // transformation
    pose_r_out = tf_mat_r * pose_r_in;

    return Mat32Quaternion(pose_r_out);
}


void FastTF::GetAngleDiff2Euler(geometry_msgs::Pose pose1_, geometry_msgs::Pose pose2_, geometry_msgs::Vector3 &angle_diff_){

    Matrix3d angle_diff_r = Matrix3d::Zero();

    Matrix3d pose1_r = Matrix3d::Zero();

    Matrix3d pose2_r = Matrix3d::Zero();

    // orientation matrix of the pose (4x1)
    pose1_r = Quaternion2Mat3(pose1_.orientation);
    pose2_r = Quaternion2Mat3(pose2_.orientation);

    // get diff
    angle_diff_r = pose1_r * pose2_r.inverse();

    angle_diff_ = Mat32Euler(angle_diff_r);
}

void FastTF::GetAngleDiff2Quaternion(geometry_msgs::Pose pose1_, geometry_msgs::Pose pose2_, geometry_msgs::Quaternion &angle_diff_){

    Matrix3d angle_diff_r = Matrix3d::Zero();

    Matrix3d pose1_r = Matrix3d::Zero();

    Matrix3d pose2_r = Matrix3d::Zero();

    // orientation matrix of the pose (4x1)
    pose1_r = Quaternion2Mat3(pose1_.orientation);
    pose2_r = Quaternion2Mat3(pose2_.orientation);

    // get diff
    angle_diff_r = pose1_r * pose2_r.inverse();

    angle_diff_ = Mat32Quaternion(angle_diff_r);
}

Matrix4d FastTF::Quaternion2Mat4(geometry_msgs::Quaternion quat_in){
    Matrix4d rotation_matrix = Matrix4d::Zero();
    double qx = quat_in.x;
    double qy = quat_in.y;
    double qz = quat_in.z;
    double qw = quat_in.w;
    double r = sqrt(pow(qx,2.0) + pow(qy,2.0) + pow(qz,2.0) + pow(qw,2.0));
    if (r!=0){
        qx = qx/r;
        qy = qy/r;
        qz = qz/r;
        qw = qw/r;
    }

    rotation_matrix(0,0) = 1 - 2*pow(qy,2) - 2*pow(qz,2);
    rotation_matrix(0,1) = 2*qx*qy - 2*qz*qw;
    rotation_matrix(0,2) = 2*qx*qz + 2*qy*qw;
    
    rotation_matrix(1,0) = 2*qx*qy + 2*qz*qw;
    rotation_matrix(1,1) = 1 - 2*pow(qx,2) - 2*pow(qz,2);
    rotation_matrix(1,2) = 2*qy*qz - 2*qx*qw;

    rotation_matrix(2,0) = 2*qx*qz - 2*qy*qw;
    rotation_matrix(2,1) = 2*qy*qz + 2*qx*qw;
    rotation_matrix(2,2) = 1 - 2*pow(qx,2) - 2*pow(qy,2);
    
    return rotation_matrix;
}

Matrix3d FastTF::Quaternion2Mat3(geometry_msgs::Quaternion quat_in){
    Matrix3d rotation_matrix = Matrix3d::Zero();
    double qx = quat_in.x;
    double qy = quat_in.y;
    double qz = quat_in.z;
    double qw = quat_in.w;

    double r = sqrt(pow(qx,2.0) + pow(qy,2.0) + pow(qz,2.0) + pow(qw,2.0));
    if (r!=0){
        qx = qx/r;
        qy = qy/r;
        qz = qz/r;
        qw = qw/r;
    }
    
    rotation_matrix(0,0) = 1 - 2*pow(qy,2) - 2*pow(qz,2);
    rotation_matrix(0,1) = 2*qx*qy - 2*qz*qw;
    rotation_matrix(0,2) = 2*qx*qz + 2*qy*qw;
    
    rotation_matrix(1,0) = 2*qx*qy + 2*qz*qw;
    rotation_matrix(1,1) = 1 - 2*pow(qx,2) - 2*pow(qz,2);
    rotation_matrix(1,2) = 2*qy*qz - 2*qx*qw;

    rotation_matrix(2,0) = 2*qx*qz - 2*qy*qw;
    rotation_matrix(2,1) = 2*qy*qz + 2*qx*qw;
    rotation_matrix(2,2) = 1 - 2*pow(qx,2) - 2*pow(qy,2);
    
    return rotation_matrix;
}

geometry_msgs::Quaternion FastTF::Mat42Quaternion(Matrix4d matrix_in){
    geometry_msgs::Quaternion quaternion;

	double r11 = matrix_in(0, 0);
	double r12 = matrix_in(0, 1);
	double r13 = matrix_in(0, 2);
	double r21 = matrix_in(1, 0);
	double r22 = matrix_in(1, 1);
	double r23 = matrix_in(1, 2);
	double r31 = matrix_in(2, 0);
	double r32 = matrix_in(2, 1);
	double r33 = matrix_in(2, 2);
	double q0 = (r11 + r22 + r33 + 1.0f) / 4.0f;
	double q1 = (r11 - r22 - r33 + 1.0f) / 4.0f;
	double q2 = (-r11 + r22 - r33 + 1.0f) / 4.0f;
	double q3 = (-r11 - r22 + r33 + 1.0f) / 4.0f;
	if (q0 < 0.0) {
		q0 = 0.0;
	}
	if (q1 < 0.0) {
		q1 = 0.0;
	}
	if (q2 < 0.0) {
		q2 = 0.0;
	}
	if (q3 < 0.0) {
		q3 = 0.0;
	}
	q0 = sqrt(q0);
	q1 = sqrt(q1);
	q2 = sqrt(q2);
	q3 = sqrt(q3);
	if (q0 >= q1 && q0 >= q2 && q0 >= q3) {
		q0 *= +1.0f;
		q1 *= ((r32 - r23) >= 0.0) ? +1.0 : -1.0;
		q2 *= ((r13 - r31) >= 0.0) ? +1.0 : -1.0;
		q3 *= ((r21 - r12) >= 0.0) ? +1.0 : -1.0;

	}
	else if (q1 >= q0 && q1 >= q2 && q1 >= q3) {
		q1 *= +1.0f;
		q0 *= ((r32 - r23) >= 0.0f) ? +1.0f : -1.0f;
		q2 *= ((r21 + r12) >= 0.0f) ? +1.0f : -1.0f;
		q3 *= ((r13 + r31) >= 0.0f) ? +1.0f : -1.0f;
	}
	else if (q2 >= q0 && q2 >= q1 && q2 >= q3) {
		q2 *= +1.0f;
		q0 *= ((r13 - r31) >= 0.0f) ? +1.0f : -1.0f;
		q1 *= ((r21 + r12) >= 0.0f) ? +1.0f : -1.0f;
		q3 *= ((r32 + r23) >= 0.0f) ? +1.0f : -1.0f;

	}
	else if (q3 >= q0 && q3 >= q1 && q3 >= q2) {
		q3 *= +1.0f;
		q0 *= ((r21 - r12) >= 0.0f) ? +1.0f : -1.0f;
		q1 *= ((r31 + r13) >= 0.0f) ? +1.0f : -1.0f;
		q2 *= ((r32 + r23) >= 0.0f) ? +1.0f : -1.0f;
	}

    double r = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q0 * q0); 

    quaternion.w = q0;
    quaternion.x = q1;
    quaternion.y = q2;
    quaternion.z = q3;

    return quaternion;
}

geometry_msgs::Quaternion FastTF::Mat32Quaternion(Matrix3d matrix_in){
    geometry_msgs::Quaternion quaternion;

	double r11 = matrix_in(0, 0);
	double r12 = matrix_in(0, 1);
	double r13 = matrix_in(0, 2);
	double r21 = matrix_in(1, 0);
	double r22 = matrix_in(1, 1);
	double r23 = matrix_in(1, 2);
	double r31 = matrix_in(2, 0);
	double r32 = matrix_in(2, 1);
	double r33 = matrix_in(2, 2);
	double q0 = (r11 + r22 + r33 + 1.0f) / 4.0f;
	double q1 = (r11 - r22 - r33 + 1.0f) / 4.0f;
	double q2 = (-r11 + r22 - r33 + 1.0f) / 4.0f;
	double q3 = (-r11 - r22 + r33 + 1.0f) / 4.0f;
	if (q0 < 0.0) {
		q0 = 0.0;
	}
	if (q1 < 0.0) {
		q1 = 0.0;
	}
	if (q2 < 0.0) {
		q2 = 0.0;
	}
	if (q3 < 0.0) {
		q3 = 0.0;
	}
	q0 = sqrt(q0);
	q1 = sqrt(q1);
	q2 = sqrt(q2);
	q3 = sqrt(q3);
	if (q0 >= q1 && q0 >= q2 && q0 >= q3) {
		q0 *= +1.0f;
		q1 *= ((r32 - r23) >= 0.0) ? +1.0 : -1.0;
		q2 *= ((r13 - r31) >= 0.0) ? +1.0 : -1.0;
		q3 *= ((r21 - r12) >= 0.0) ? +1.0 : -1.0;

	}
	else if (q1 >= q0 && q1 >= q2 && q1 >= q3) {
		q1 *= +1.0f;
		q0 *= ((r32 - r23) >= 0.0f) ? +1.0f : -1.0f;
		q2 *= ((r21 + r12) >= 0.0f) ? +1.0f : -1.0f;
		q3 *= ((r13 + r31) >= 0.0f) ? +1.0f : -1.0f;
	}
	else if (q2 >= q0 && q2 >= q1 && q2 >= q3) {
		q2 *= +1.0f;
		q0 *= ((r13 - r31) >= 0.0f) ? +1.0f : -1.0f;
		q1 *= ((r21 + r12) >= 0.0f) ? +1.0f : -1.0f;
		q3 *= ((r32 + r23) >= 0.0f) ? +1.0f : -1.0f;

	}
	else if (q3 >= q0 && q3 >= q1 && q3 >= q2) {
		q3 *= +1.0f;
		q0 *= ((r21 - r12) >= 0.0f) ? +1.0f : -1.0f;
		q1 *= ((r31 + r13) >= 0.0f) ? +1.0f : -1.0f;
		q2 *= ((r32 + r23) >= 0.0f) ? +1.0f : -1.0f;
	}

    double r = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q0 * q0); 

    quaternion.w = q0;
    quaternion.x = q1;
    quaternion.y = q2;
    quaternion.z = q3;

    return quaternion;
}

geometry_msgs::Vector3 FastTF::Mat32Euler(Matrix3d matrix_in){
    geometry_msgs::Vector3 euler;
    tf::Matrix3x3 m(matrix_in(0,0), matrix_in(0,1), matrix_in(0,2), 
                    matrix_in(1,0), matrix_in(1,1), matrix_in(1,2),
                    matrix_in(2,0), matrix_in(2,1), matrix_in(2,2));
    double r, p, y;
    m.getRPY(r, p, y);
    euler.x = r;
    euler.y = p;
    euler.z = y;

    return euler;
}