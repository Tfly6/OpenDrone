/***************************************************************************************************************************
* math_utils.h
*
* Author: ref to Qyp
***************************************************************************************************************************/
#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <Eigen/Eigen>
#include <math.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>


// 四元数转欧拉角
Eigen::Vector3d quaternion_to_rpy2(const Eigen::Quaterniond &q)
{
    // YPR - ZYX
    return q.toRotationMatrix().eulerAngles(2, 1, 0).reverse();
}

// 从(roll,pitch,yaw)创建四元数  by a 3-2-1 intrinsic Tait-Bryan rotation sequence
Eigen::Quaterniond quaternion_from_rpy(const Eigen::Vector3d &rpy)
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
Eigen::Vector3d quaternion_to_euler(const Eigen::Quaterniond &q)
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
Eigen::Vector3d rotation_to_euler(Eigen::Matrix3d rotation)
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

// geometry_msgs to Eigen
inline Eigen::Vector3d toEigen(const geometry_msgs::Point &p) {

    Eigen::Vector3d ev3(p.x, p.y, p.z);
    return ev3;
}

inline Eigen::Vector3d toEigen(const geometry_msgs::Vector3 &v3) {
    Eigen::Vector3d ev3(v3.x, v3.y, v3.z);
    return ev3;
}

inline geometry_msgs::Point toGeometryMsg(const Eigen::Vector3d &v3) {
    geometry_msgs::Point p;
    p.x = v3(0);
    p.y = v3(1);
    p.z = v3(2);
    return p;
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
float constrain_function(float data, float Max)
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

//constrain_function2
float constrain_function2(float data, float Min,float Max)
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
float sign_function(float data)
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

// min function
float min(float data1,float data2)
{
    if(data1>=data2)
    {
        return data2;
    }
    else
    {
        return data1;
    }
}

#endif

