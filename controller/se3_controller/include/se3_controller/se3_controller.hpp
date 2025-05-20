#ifndef SE3_CONTROLLER_HPP
#define SE3_CONTROLLER_HPP

#include <queue>
#include <Eigen/Dense>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include "se3_controller/utils.hpp"

#define VEL_IN_BODY /* cancel the comment if the velocity in odom topic is relative to current body frame, not to world frame.*/
// #define AIRSIM

struct Odom_Data_t{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Eigen::Vector3d p;
	Eigen::Vector3d v;
	Eigen::Quaterniond q;
	Eigen::Vector3d w;

	nav_msgs::Odometry msg;
	ros::Time rcv_stamp;
	bool recv_new_msg;

	Odom_Data_t(){
		recv_new_msg = false;
	}
	void feed(nav_msgs::OdometryConstPtr pMsg, bool enu_frame, bool vel_in_body){
		msg = *pMsg;
		rcv_stamp = ros::Time::now();
		recv_new_msg = true;

		p(0) = msg.pose.pose.position.x;
		p(1) = msg.pose.pose.position.y;
		p(2) = msg.pose.pose.position.z;

		v(0) = msg.twist.twist.linear.x;
		v(1) = msg.twist.twist.linear.y;
		v(2) = msg.twist.twist.linear.z;

		q.w() = msg.pose.pose.orientation.w;
		q.x() = msg.pose.pose.orientation.x;
		q.y() = msg.pose.pose.orientation.y;
		q.z() = msg.pose.pose.orientation.z;

		w(0) = msg.twist.twist.angular.x;
		w(1) = msg.twist.twist.angular.y;
		w(2) = msg.twist.twist.angular.z;

		if(!enu_frame){
			Eigen::Matrix3d R_mid;
			R_mid << 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0;
			Eigen::Quaterniond q_mid(R_mid);

			p = q_mid.toRotationMatrix() * p;
			v = q_mid.toRotationMatrix() * v;
			q = q_mid * q * q_mid;
			q.normalize();
			w = q_mid.toRotationMatrix() * w;
		}

		if(vel_in_body)
			v = q.toRotationMatrix() * v;
	}
};

struct Desired_State_t
{
	Eigen::Vector3d p;
	Eigen::Vector3d v;
	Eigen::Vector3d a;
	Eigen::Vector3d j;
	Eigen::Quaterniond q;
	double yaw;
	double yaw_rate;

	Desired_State_t(){
		p = Eigen::Vector3d::Zero();
		v = Eigen::Vector3d::Zero();
		a = Eigen::Vector3d::Zero();
		j = Eigen::Vector3d::Zero();
		q.w() = 1;
		q.x() = 0;
		q.y() = 0;
		q.z() = 0;
		yaw = 0;
		yaw_rate = 0;
	}

	Desired_State_t(Odom_Data_t odom){
		p = odom.p;
		v = Eigen::Vector3d::Zero();
		a = Eigen::Vector3d::Zero();
		j = Eigen::Vector3d::Zero();
		q = odom.q.normalized();
		yaw = utils::fromQuaternion2yaw(q);
		yaw_rate = 0;
	}
};

struct Controller_Output_t
{
	// Eigen::Vector3d v;

	// Orientation of the body frame with respect to the world frame
	Eigen::Quaterniond q;

	// Body rates in body frame
	Eigen::Vector3d bodyrates; // [rad/s]

	// Collective mass normalized thrust
	double thrust;
};

struct Imu_Data_t{
	Eigen::Quaterniond q;
	Eigen::Vector3d w;
	Eigen::Vector3d a;

	sensor_msgs::Imu msg;
	ros::Time rcv_stamp;
	bool recv_new_msg;

	Imu_Data_t(){
		recv_new_msg = false;
	}
	void feed(sensor_msgs::ImuConstPtr pMsg, bool enu_frame){
		msg = *pMsg;
		rcv_stamp = ros::Time::now();
		recv_new_msg = true;

		a(0) = msg.linear_acceleration.x;
		a(1) = msg.linear_acceleration.y;
		a(2) = msg.linear_acceleration.z;

		q.x() = msg.orientation.x;
		q.y() = msg.orientation.y;
		q.z() = msg.orientation.z;
		q.w() = msg.orientation.w;

		w(0) = msg.angular_velocity.x;
		w(1) = msg.angular_velocity.y;
		w(2) = msg.angular_velocity.z;

		if(!enu_frame){
			Eigen::Matrix3d R_mid;
			R_mid << 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0;
			Eigen::Quaterniond q_mid(R_mid);

			a = q_mid.toRotationMatrix() * a;
			q = q_mid * q * q_mid;
			q.normalize();
			w = q_mid.toRotationMatrix() * w;
		}

		// check the frequency
		// static int one_min_count = 9999;
		// static ros::Time last_clear_count_time = ros::Time(0.0);
		// if ( (now - last_clear_count_time).toSec() > 1.0 ){
		// 	if ( one_min_count < 100 )
		// 		ROS_WARN("IMU frequency seems lower than 100Hz, which is too low!");
		// 	one_min_count = 0;
		// 	last_clear_count_time = now;
		// }
		// one_min_count ++;
	}
};

class SE3_CONTROLLER
{
private:
	Eigen::Vector3d Kp_p_, Kp_v_, Kp_a_, Kp_q_, Kp_w_, Kd_p_, Kd_v_, Kd_a_, Kd_q_, Kd_w_;
	double limit_err_p_, limit_err_v_, limit_err_a_, limit_d_err_p_, limit_d_err_v_, limit_d_err_a_;
	bool have_last_err_, enu_frame_, vel_in_body_;

	double hover_percent_, max_hover_percent_;
	double T_a_; // normalization constant
	double P_ = 1e6;
	const double rho_ = 0.998; // confidence
	const double gravity_ = 9.81;
	static constexpr double kAlmostZeroValueThreshold_ = 0.001;
	Eigen::Vector3d grav_vec_, last_err_p_, last_err_v_, last_err_a_, last_err_q_, last_err_w_;
	std::queue<std::pair<ros::Time, double>> timed_thrust_;

	void computeFlatInput(Desired_State_t desired_state, Odom_Data_t &desired_odom){
		
		desired_odom.p = desired_state.p;
		desired_odom.v = desired_state.v;

		// TODO check
		Eigen::Vector3d xc(cos(desired_state.yaw), sin(desired_state.yaw), 0);
		Eigen::Vector3d yc(-sin(desired_state.yaw), cos(desired_state.yaw), 0);

		Eigen::Vector3d zb = desired_state.a.normalized();
		Eigen::Vector3d xb = yc.cross(zb);
		xb.normalize();
		Eigen::Vector3d yb = zb.cross(xb);
		yb.normalize();
		
		// Eigen::Vector3d xb = yc.cross(desired_state.a);
		// xb.normalize();
		// Eigen::Vector3d yb = desired_state.a.cross(xb);
		// yb.normalize();
		// Eigen::Vector3d zb = xb.cross(yb);
		Eigen::Matrix3d rotM;
		rotM << xb, yb, zb;
		desired_odom.q = Eigen::Quaterniond(rotM);

		double a_zb = zb.dot(desired_state.a);
		desired_odom.w(0) = -yb.dot(desired_state.j) / a_zb;
		desired_odom.w(1) = xb.dot(desired_state.j) / a_zb;
		desired_odom.w(2) = desired_state.yaw_rate * xc.dot(xb) + yc.dot(zb) * desired_odom.w(1);
		desired_odom.w(2) /= (yc.cross(zb)).norm();
	}

	void computeFlatInput_Hopf_Fibration(Desired_State_t desired_state, Odom_Data_t &desired_odom){
		Eigen::Vector3d abc = desired_state.a.normalized();
		double a = abc(0), b = abc(1), c = abc(2);
		Eigen::Vector3d abc_dot = (desired_state.a.dot(desired_state.a) * Eigen::MatrixXd::Identity(3, 3) - desired_state.a * desired_state.a.transpose()) / desired_state.a.norm() / desired_state.a.squaredNorm() * desired_state.j;
		double a_dot = abc_dot(0), b_dot = abc_dot(1), c_dot = abc_dot(2);
		double yaw = desired_state.yaw;
		double yaw_dot = desired_state.yaw_rate;
		double syaw = sin(yaw), cyaw = cos(yaw);

		if(c > 0){
			double norm = sqrt(2 * (1 + c));
			Eigen::Quaterniond q((1 + c) / norm, -b / norm, a / norm, 0);
			Eigen::Quaterniond q_yaw(cos(yaw / 2), 0, 0, sin(yaw / 2));
			desired_odom.q = q * q_yaw;
			desired_odom.w(0) = syaw * a_dot - cyaw * b_dot - (a * syaw - b * cyaw) * c_dot / (c + 1);
			desired_odom.w(1) = cyaw * a_dot + syaw * b_dot - (a * cyaw + b * syaw) * c_dot / (c + 1);
			desired_odom.w(2) = (b * a_dot - a * b_dot) / (1 + c) + yaw_dot;
		}else{
			double norm = sqrt(2 * (1 - c));
			Eigen::Quaterniond q(-b / norm, (1 - c) / norm, 0, a / norm);
			yaw += 2 * atan2(a, b);
			Eigen::Quaterniond q_yaw(cos(yaw / 2), 0, 0, sin(yaw / 2));
			desired_odom.q = q * q_yaw;
			syaw = sin(yaw);
			cyaw = cos(yaw);
			yaw_dot = yaw_dot;// + atan2_dot(a, b, a_dot, b_dot);
			
			desired_odom.w(0) = syaw * a_dot + cyaw * b_dot - (a * syaw + b * cyaw) * c_dot / (c - 1);
			desired_odom.w(1) = cyaw * a_dot - syaw * b_dot - (a * cyaw - b * syaw) * c_dot / (c - 1);
			desired_odom.w(2) = (b * a_dot - a * b_dot) / (c - 1) + yaw_dot;
		}
	}

	void limitErr(Eigen::Vector3d &err, double low, double upper){
		err(0) = std::max(std::min(err(0), upper), low);
		err(1) = std::max(std::min(err(1), upper), low);
		err(2) = std::max(std::min(err(2), upper), low);
	}

	double limitYaw(double yaw_curr, double yaw_des, double yaw_limit){
		double err = yaw_des - yaw_curr;
		yaw_limit = std::abs(yaw_limit);
		if(err < -yaw_limit){
			return yaw_curr - yaw_limit;
		}else if (err > yaw_limit){
			return yaw_curr + yaw_limit;
		}else{
			return yaw_des;
		}
	}

public:
	SE3_CONTROLLER(){};
    ~SE3_CONTROLLER(){};

	void init(double hover_percent, double max_hover_percent, bool enu_frame, bool vel_in_body){
		
		hover_percent_ = hover_percent;
		max_hover_percent_ = max_hover_percent;
		enu_frame_ = enu_frame;
		vel_in_body_ = vel_in_body;
		T_a_ = gravity_ / hover_percent_;
		grav_vec_ << 0.0, 0.0, gravity_;

		last_err_p_ = Eigen::Vector3d::Zero();
		last_err_v_ = Eigen::Vector3d::Zero();
		last_err_a_ = Eigen::Vector3d::Zero();
		last_err_q_ = Eigen::Vector3d::Zero();
		last_err_w_ = Eigen::Vector3d::Zero();

		have_last_err_ = false;
	}

	void setup(Eigen::Vector3d kp_p, Eigen::Vector3d kp_v, Eigen::Vector3d kp_a, Eigen::Vector3d kp_q, Eigen::Vector3d kp_w, 
				Eigen::Vector3d kd_p, Eigen::Vector3d kd_v, Eigen::Vector3d kd_a, Eigen::Vector3d kd_q, Eigen::Vector3d kd_w,
				double limit_err_p, double limit_err_v, double limit_err_a, 
				double limit_d_err_p, double limit_d_err_v, double limit_d_err_a){
		Kp_p_ = kp_p;
		Kp_v_ = kp_v;
		Kp_a_ = kp_a;
		Kp_q_ = kp_q;
		Kp_w_ = kp_w;
		Kd_p_ = kd_p;
		Kd_v_ = kd_v;
		Kd_a_ = kd_a;
		Kd_q_ = kd_q;
		Kd_w_ = kd_w;
		limit_err_p_ = limit_err_p;
		limit_err_v_ = limit_err_v;
		limit_err_a_ = limit_err_a;
		limit_d_err_p_ = limit_d_err_p;
		limit_d_err_v_ = limit_d_err_v;
		limit_d_err_a_ = limit_d_err_a;
	}

	bool calControl(Odom_Data_t odom_data, Imu_Data_t imu_data, Desired_State_t desired_state, Controller_Output_t &output){
		if((ros::Time::now() - odom_data.rcv_stamp).toSec() > 0.1){
			// std::cout << "odom not rcv" << std::endl;
			return false;
		}
		// desired_state.yaw = limitYaw(fromQuaternion2yaw(odom_data.q), desired_state.yaw, M_PI_2 / 3);
		Eigen::Vector3d err_p = odom_data.p - desired_state.p;
		limitErr(err_p, -limit_err_p_, limit_err_p_);
		if(have_last_err_ == false)
			last_err_p_ = err_p;
		Eigen::Vector3d d_err_p = err_p - last_err_p_;
		limitErr(d_err_p, -limit_d_err_p_, limit_d_err_p_);
		desired_state.v = desired_state.v - Kp_p_.asDiagonal() * err_p - Kd_p_.asDiagonal() * d_err_p;
		Eigen::Vector3d err_v = odom_data.v - desired_state.v;
		limitErr(err_v, -limit_err_v_, limit_err_v_);
		if(have_last_err_ == false)
			last_err_v_ = err_v;
		Eigen::Vector3d d_err_v = err_v - last_err_v_;
		limitErr(d_err_v, -limit_d_err_v_, limit_d_err_v_);
		desired_state.a = desired_state.a - Kp_v_.asDiagonal() * err_v - Kd_v_.asDiagonal() * d_err_v + grav_vec_;
		// std::cout << "err_p: " << err_p.transpose() << std::endl;
		// std::cout << "err_v: " << err_v.transpose() << std::endl;
		// std::cout << "imu_data.a: " << imu_data.a.transpose() << std::endl;
		// std::cout << "odom_data.v: " << odom_data.v.transpose() << std::endl;
		Eigen::Vector3d a_world = odom_data.q.toRotationMatrix() * imu_data.a;
		Eigen::Vector3d err_a = a_world - desired_state.a;
		limitErr(err_a, -limit_err_a_, limit_err_a_);
		if(have_last_err_ == false)
			last_err_a_ = err_a;
		Eigen::Vector3d d_err_a = err_a - last_err_a_;
		limitErr(d_err_a, -limit_d_err_a_, limit_d_err_a_);
		desired_state.j = desired_state.j - Kp_a_.asDiagonal() * err_a - Kd_a_.asDiagonal() * d_err_a;

		last_err_p_ = err_p;
		last_err_v_ = err_v;
		last_err_a_ = err_a;
		
		double thr = desired_state.a.transpose() * (odom_data.q * Eigen::Vector3d::UnitZ());
		output.thrust = thr / T_a_;
		// std::cout << std::endl << "desired_state.a: " << desired_state.a.transpose() << std::endl;
		// std::cout << "odom_v: " << odom_data.v.transpose() << std::endl;
		
		Odom_Data_t desired_odom;
		// computeFlatInput(desired_state, desired_odom);
		computeFlatInput_Hopf_Fibration(desired_state, desired_odom);
		output.q = imu_data.q * odom_data.q.inverse() * desired_odom.q; // Align with FCU frame, from odom to imu frame
		
		// printf("desired q: (%lf,%lf,%lf,%lf)\n", desired_odom.q.w(), desired_odom.q.x(), desired_odom.q.y(), desired_odom.q.z());
		// std::cout << "desired q: " << desired_state.a.transpose() << std::endl;

		Eigen::Quaterniond err_q = odom_data.q.inverse() * desired_odom.q;
		Eigen::Vector3d err_br;
		if (err_q.w() >= 0){
			err_br.x() = Kp_q_(0) * err_q.x();
			err_br.y() = Kp_q_(1) * err_q.y();
			err_br.z() = Kp_q_(2) * err_q.z();
		}
		else{
			err_br.x() = -Kp_q_(0) * err_q.x();
			err_br.y() = -Kp_q_(1) * err_q.y();
			err_br.z() = -Kp_q_(2) * err_q.z();
		}

		output.bodyrates = desired_odom.w + err_br;

		if(!enu_frame_){
			Eigen::Matrix3d R_mid;
			R_mid << 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0;
			Eigen::Quaterniond q_mid(R_mid.inverse());
			output.q = q_mid * output.q * q_mid;
			output.bodyrates = q_mid * output.bodyrates;
		}
	
		// Eigen::Quaterniond err_q = odom_data.q * (desired_odom.q.inverse());
		// // limitErr(err_q, -1.0, 1.0);
		// // Eigen::Vector3d err_w = odom_data.w - odom_data.q.matrix().transpose() * desired_odom.q.matrix() * desired_odom.w;
		// Eigen::Vector3d err_w = odom_data.w - desired_odom.w;
		// limitErr(err_w, -1.0, 1.0);
		// if(have_last_err_ == false){
		// 	have_last_err_ = true;
		// 	last_err_q_ = err_q.vec();
		// 	last_err_w_ = err_w;
		// }
		// Eigen::Vector3d d_err_q = err_q.vec() - last_err_q_;
		// limitErr(d_err_q, -1.0, 1.0);
		// Eigen::Vector3d d_err_w = err_w - last_err_w_;
		// limitErr(d_err_w, -1.0, 1.0);
		// if (err_q.w() >= 0){
		// 	output.bodyrates = desired_odom.w - Kp_q_.asDiagonal() * err_q.vec() - Kp_w_.asDiagonal() * err_w - Kd_q_.asDiagonal() * d_err_q - Kd_w_.asDiagonal() * d_err_w;
		// 	// err_br.x() = Kp_q_(0) * err_q.x();
		// 	// err_br.y() = Kp_q_(1) * err_q.y();
		// 	// err_br.z() = Kp_q_(2) * err_q.z();
		// }
		// else{
		// 	output.bodyrates = desired_odom.w + Kp_q_.asDiagonal() * err_q.vec() - Kp_w_.asDiagonal() * err_w + Kd_q_.asDiagonal() * d_err_q - Kd_w_.asDiagonal() * d_err_w;
		// 	// err_br.x() = -Kp_q_(0) * err_q.x();
		// 	// err_br.y() = -Kp_q_(1) * err_q.y();
		// 	// err_br.z() = -Kp_q_(2) * err_q.z();
		// }
		// // std::cout << "thrust: " << output.thrust << std::endl;
		// // std::cout << "bodyrates: " << output.bodyrates.transpose() << std::endl;
		// last_err_q_ = err_q.vec();
		// last_err_w_ = err_w;

		timed_thrust_.push(std::pair<ros::Time, double>(ros::Time::now(), output.thrust));
		while (timed_thrust_.size() > 100)
			timed_thrust_.pop();
		return true;
	}

	bool estimateTa(const Eigen::Vector3d &est_a){
		ros::Time t_now = ros::Time::now();
		while (timed_thrust_.size() >= 1)
		{
			// Choose data before 35~45ms ago
			std::pair<ros::Time, double> t_t = timed_thrust_.front();
			double time_passed = (t_now - t_t.first).toSec();
			if (time_passed > 0.045){ // 45ms
				timed_thrust_.pop();
				continue;
			}
			if (time_passed < 0.035){ // 35ms
				return false;
			}

			/***********************************************************/
			/* Recursive least squares algorithm with vanishing memory */
			/***********************************************************/
			double thr = t_t.second;
			timed_thrust_.pop();
			
			/***********************************/
			/* Model: est_a(2) = thr1acc_ * thr */
			/***********************************/
			double gamma = 1 / (rho_ + thr * P_ * thr);
			double K = gamma * P_ * thr;
			T_a_ = T_a_ + K * (est_a(2) - thr * T_a_);
			P_ = (1 - K * thr) * P_ / rho_;
			T_a_ = std::max(T_a_, gravity_ / max_hover_percent_);
			// printf("%6.3f,%6.3f,%6.3f,%6.3f\n", T_a_, gamma, K, P_);
			//fflush(stdout);

			return true;
		}
		return false;
	}
};

#endif