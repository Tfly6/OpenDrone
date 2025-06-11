/***************************************************************************************************************************
* math_utils.h
* Author: TFly
***************************************************************************************************************************/
#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <Eigen/Eigen>
#include <math.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>

using namespace std;

// 四元数转欧拉角
inline Eigen::Vector3d quaternion_to_rpy2(const Eigen::Quaterniond &q)
{
    // YPR - ZYX
    return q.toRotationMatrix().eulerAngles(2, 1, 0).reverse();
}

// 从(roll,pitch,yaw)创建四元数  by a 3-2-1 intrinsic Tait-Bryan rotation sequence
inline Eigen::Quaterniond quaternion_from_rpy(const Eigen::Vector3d &rpy)
{
    // YPR - ZYX
    return Eigen::Quaterniond(
                Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX())
                );
}



// 将四元数转换至(roll,pitch,yaw)  by a 3-2-1 intrinsic Tait-Bryan rotation sequence
// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
// q0 q1 q2 q3
// w x y z
inline Eigen::Vector3d quaternion_to_euler(const Eigen::Quaterniond &q)
{
    float quat[4];
    quat[0] = q.w();
    quat[1] = q.x();
    quat[2] = q.y();
    quat[3] = q.z();

    Eigen::Vector3d ans;
    ans[0] = atan2(2.0 * (quat[3] * quat[2] + quat[0] * quat[1]), 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]));
    ans[1] = asin(2.0 * (quat[2] * quat[0] - quat[3] * quat[1]));
    ans[2] = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), 1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3]));
    return ans;
}

//旋转矩阵转欧拉角
inline Eigen::Vector3d rotation_to_euler(Eigen::Matrix3d rotation)
{
    Eigen::Vector3d euler_angle;

    double phi_val = atan2(rotation(2, 1), rotation(2, 2));
    double theta_val = asin(-rotation(2, 0));
    double psi_val = atan2(rotation(1, 0), rotation(0, 0));
    double pi = M_PI;

    if (fabs(theta_val - pi / 2.0) < 1.0e-3) {
        phi_val = 0.0;
        psi_val = atan2(rotation(1, 2), rotation(0, 2));

    } else if (fabs(theta_val + pi / 2.0) <  1.0e-3) {
        phi_val = 0.0;
        psi_val = atan2(-rotation(1, 2), -rotation(0, 2));
    }

    euler_angle(0) = phi_val;
    euler_angle(1) = theta_val;
    euler_angle(2) = psi_val;

    return euler_angle;
}

//旋转矩阵转四元数
inline Eigen::Vector4d rot2Quaternion(const Eigen::Matrix3d &R) {
    Eigen::Vector4d quat;
    double tr = R.trace();
    if (tr > 0.0) {
      double S = sqrt(tr + 1.0) * 2.0;  // S=4*qw
      quat(0) = 0.25 * S;
      quat(1) = (R(2, 1) - R(1, 2)) / S;
      quat(2) = (R(0, 2) - R(2, 0)) / S;
      quat(3) = (R(1, 0) - R(0, 1)) / S;
    } else if ((R(0, 0) > R(1, 1)) & (R(0, 0) > R(2, 2))) {
      double S = sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2)) * 2.0;  // S=4*qx
      quat(0) = (R(2, 1) - R(1, 2)) / S;
      quat(1) = 0.25 * S;
      quat(2) = (R(0, 1) + R(1, 0)) / S;
      quat(3) = (R(0, 2) + R(2, 0)) / S;
    } else if (R(1, 1) > R(2, 2)) {
      double S = sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2)) * 2.0;  // S=4*qy
      quat(0) = (R(0, 2) - R(2, 0)) / S;
      quat(1) = (R(0, 1) + R(1, 0)) / S;
      quat(2) = 0.25 * S;
      quat(3) = (R(1, 2) + R(2, 1)) / S;
    } else {
      double S = sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1)) * 2.0;  // S=4*qz
      quat(0) = (R(1, 0) - R(0, 1)) / S;
      quat(1) = (R(0, 2) + R(2, 0)) / S;
      quat(2) = (R(1, 2) + R(2, 1)) / S;
      quat(3) = 0.25 * S;
    }
    return quat;
}

// geometry_msgs to Eigen
inline Eigen::Vector3d toEigen(const geometry_msgs::Point &p) {

    Eigen::Vector3d ev3(p.x, p.y, p.z);
    return ev3;
}

inline Eigen::Vector3d toEigen(const geometry_msgs::Vector3 &v3) {
    Eigen::Vector3d ev3(v3.x, v3.y, v3.z);
    return ev3;
}

//  Eigen to geometry_msgs
template <typename T>
T toGeometryMsg(const Eigen::Vector3d &v3) {
    T res;
    res.x = v3(0);
    res.y = v3(1);
    res.z = v3(2);
    return res;
}

inline double deg_to_rad(const double degrees) {
    return degrees * M_PI / 180.0;
}

inline Eigen::Vector3d deg_to_rad(const Eigen::Vector3d &degrees) {
    
	Eigen::Vector3d rad(deg_to_rad(degrees[0]), deg_to_rad(degrees[1]), deg_to_rad(degrees[2]));
	
	return rad;
}

inline double rad_to_deg(double radians) {
    return radians * 180.0 / M_PI;
}

inline Eigen::Vector3d rad_to_deg(const Eigen::Vector3d &radians) {
    
	Eigen::Vector3d deg(rad_to_deg(radians[0]), rad_to_deg(radians[1]), rad_to_deg(radians[2]));
	
	return deg;
}
//constrain_function
inline double constrain_function(double data, double Max)
{
    if(abs(data)>Max)
    {
        return (data > 0) ? Max : -Max;
    }
    else
    {
        return data;
    }
}

inline void satura(Eigen::Vector3d &err, double low, double upper){
    err(0) = std::max(std::min(err(0), upper), low);
    err(1) = std::max(std::min(err(1), upper), low);
    err(2) = std::max(std::min(err(2), upper), low);
}


inline double satura(double data, double Min,double Max)
{
    if(data > Max)
    {
        return Max;
    }
    else if (data < Min)
    {
        return Min;
    }else
    {
        return data;
    }
}

//sign_function
inline double sign(double data)
{
    if(data>0)
    {
        return 1.0;
    }
    else if(data<0)
    {
        return -1.0;
    }
    else if(data == 0)
    {
        return 0.0;
    }
}

inline bool is_arrive(const geometry_msgs::PoseStamped &pos1, const geometry_msgs::PoseStamped &pos2){
    double distance_threshold = 0.13;  // 到达阈值
    double dx = pos1.pose.position.x - pos2.pose.position.x;
    double dy = pos1.pose.position.y - pos2.pose.position.y;
    double dz = pos1.pose.position.z - pos2.pose.position.z;
    double distance = sqrt(dx*dx + dy*dy + dz*dz);
    return distance < distance_threshold;
}

inline bool is_arrive(const Eigen::Vector3d &pos1, const Eigen::Vector3d &pos2){
    double distance_threshold = 0.13;  // 到达阈值
    double dx = pos1[0] - pos2[0];
    double dy = pos1[1] - pos2[1];
    double dz = pos1[2] - pos2[2];
    double distance = sqrt(dx*dx + dy*dy + dz*dz);
    return distance < distance_threshold;
}

#endif

