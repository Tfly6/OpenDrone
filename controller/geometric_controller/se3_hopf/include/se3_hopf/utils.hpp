#ifndef SE3_CONTROLLER_UTILS_HPP
#define SE3_CONTROLLER_UTILS_HPP

#include <Eigen/Dense>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace utils{
	inline double fromQuaternion2yaw(Eigen::Quaterniond q){
		return atan2(2 * (q.x()*q.y() + q.w()*q.z()), q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());
	}

	inline Eigen::Vector3d quat2euler(Eigen::Quaterniond q){
		tf2::Quaternion quat_tf;
		geometry_msgs::Quaternion quat_msg;
		quat_msg.w = q.w();
		quat_msg.x = q.x();
		quat_msg.y = q.y();
		quat_msg.z = q.z();
		double roll, pitch, yaw;
		tf2::fromMsg(quat_msg, quat_tf);
		tf2::Matrix3x3(quat_tf).getRPY(roll, pitch, yaw);
		Eigen::Vector3d euler(roll, pitch, yaw);
		return euler;
	}

	inline Eigen::Quaterniond euler2quat(double roll, double pitch, double yaw){
		tf2::Quaternion orientation;
		orientation.setRPY(roll, pitch, yaw);
		Eigen::Quaterniond q(orientation.getW(), orientation.getX(), orientation.getY(), orientation.getZ());
		return q;
	}

	inline double atan2_dot(double y, double x, double y_dot, double x_dot){
		if(x == 0.0 && y == 0.0){
			return 0;
		}else{
			// return 1 / (1 + (y / x) * (y / x)) * (y_dot * x - y * x_dot) / (x * x);
			return (y_dot * x - y * x_dot) / (x * x + y * y);
		}
	}
} // namespace utils



#endif