#ifndef CASCADE_PID_H
#define CASCADE_PID_H

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>

#include "math_utils/math_utils.h"
using namespace std;

class cascadePID {
    public:
        cascadePID(void):nh_("~"){

            nh_.param<double>("cascade_pid/Kp_px", kp_p_[0], 1.0);
            nh_.param<double>("cascade_pid/Kp_py", kp_p_[1], 1.0);
            nh_.param<double>("cascade_pid/Kp_pz" , kp_p_[2], 2.0);
            nh_.param<double>("cascade_pid/Kd_px", kd_p_[0], 0.5);
            nh_.param<double>("cascade_pid/Kd_py", kd_p_[1], 0.5);
            nh_.param<double>("cascade_pid/Kd_pz" , kd_p_[2], 0.5);
            nh_.param<double>("cascade_pid/Ki_px", ki_p_[0], 0.2);
            nh_.param<double>("cascade_pid/Ki_py", ki_p_[1], 0.2);
            nh_.param<double>("cascade_pid/Ki_pz" , ki_p_[2], 0.2);

            nh_.param<double>("cascade_pid/Kp_vx", kp_v_[0], 1.0);
            nh_.param<double>("cascade_pid/Kp_vy", kp_v_[1], 1.0);
            nh_.param<double>("cascade_pid/Kp_vz" , kp_v_[2], 2.0);
            nh_.param<double>("cascade_pid/Kd_vx", kd_v_[0], 0.5);
            nh_.param<double>("cascade_pid/Kd_vy", kd_v_[1], 0.5);
            nh_.param<double>("cascade_pid/Kd_vz" , kd_v_[2], 0.5);
            nh_.param<double>("cascade_pid/Ki_vx", ki_v_[0], 0.2);
            nh_.param<double>("cascade_pid/Ki_vy", ki_v_[1], 0.2);
            nh_.param<double>("cascade_pid/Ki_vz" , ki_v_[2], 0.2);

            nh_.param<double>("cascade_pid/maxVel", maxVel_, 2.0);
            nh_.param<double>("cascade_pid/maxAcc", maxAcc_, 3.0);
            nh_.param<double>("cascade_pid/pos_err_max", pos_error_max_, 0.8);
            nh_.param<double>("cascade_pid/pos_integral_max", pos_integral_max_, 0.8);
            nh_.param<double>("cascade_pid/vel_err_max", vel_error_max_, 0.8);
            nh_.param<double>("cascade_pid/vel_integral_max", vel_integral_max_, 0.8);
            
            integral_p_ <<0, 0, 0;
            pre_error_p_ <<0, 0, 0;

            integral_v_ <<0, 0, 0;
            pre_error_v_ <<0, 0, 0;

        }

        Eigen::Vector4d compute(const Eigen::Vector3d &currPose, const Eigen::Vector3d &currVel, 
            const Eigen::Vector3d &targetPose, double yaw_ref, double dt) {

            // 位置环输出为速度指令
            Eigen::Vector3d pos_error = targetPose - currPose;
            satura(pos_error, -pos_error_max_, pos_error_max_);

            // integral_p_ += pos_error * dt;
            // satura(integral_p_, -pos_integral_max_, pos_integral_max_);

            // Eigen::Vector3d derivative_p = (pos_error - pre_error_p_) / dt;
            // pre_error_p_ = pos_error;

            Eigen::Vector3d target_vel;
            target_vel = kp_p_.asDiagonal() * pos_error;
            // target_vel = pid(targetPose, currPose, kd_p_, ki_p_, kd_p_, integral_p_, pre_error_p_,
            //                     pos_integral_max_, pos_error_max_, dt);
            
            // target_vel = kp_p_.asDiagonal() * pos_error + ki_p_.asDiagonal() * integral_ + kd_p_.asDiagonal() * derivative_p;

            // 速度环输出为加速度指令
            // Eigen::Vector3d vel_error = target_vel - currVel;
            // integral_v_ += vel_error * dt;
            // Eigen::Vector3d derivative_v = (vel_error - pre_error_v_) / dt;
            // pre_error_v_ = vel_error;

            Eigen::Vector3d target_acc;
            target_acc = pid(target_vel, currVel, kp_v_, ki_v_, kd_v_, integral_v_, pre_error_v_,
                                vel_integral_max_ ,vel_error_max_, dt);
            // target_acc = kp_v_.asDiagonal() * vel_error + ki_v_.asDiagonal() * integral_v_ + kd_v_.asDiagonal() * derivative_v;
            target_acc[2] += GRAVITY;
            

            Eigen::Vector4d target_att = acc2quaternion(target_acc, yaw_ref);

            // 推力计算
            // if(target_acc.norm() > maxAcc_){
            //     target_acc = (maxAcc_ / target_acc.norm()) * target_acc;
            // }
            thrust_des_ = (target_acc[2] * 0.70) / GRAVITY; // 归一化

            return target_att; 
        }

        Eigen::Vector3d pid(Eigen::Vector3d target, Eigen::Vector3d curr, Eigen::Vector3d p, Eigen::Vector3d i, Eigen::Vector3d d,
            Eigen::Vector3d &integral, Eigen::Vector3d &pre_error, double integral_max, double err_max, double dt){
            
            Eigen::Vector3d error = target - curr;
            satura(error, -err_max, err_max);

            integral += error * dt;
            satura(integral, -integral_max, integral_max);

            Eigen::Vector3d derivative = (error - pre_error) / dt;
            pre_error = error;

            Eigen::Vector3d des;
            des = p.asDiagonal() * error + i.asDiagonal() * integral + d.asDiagonal() * derivative;

            return des;
        }

        Eigen::Vector4d acc2quaternion(const Eigen::Vector3d &vector_acc, const double &yaw) {
            Eigen::Vector4d quat;
            Eigen::Vector3d zb_des, yb_des, xb_des, proj_xb_des;
            Eigen::Matrix3d rotmat;
          
            proj_xb_des << std::cos(yaw), std::sin(yaw), 0.0;
          
            zb_des = vector_acc / vector_acc.norm();
            yb_des = zb_des.cross(proj_xb_des) / (zb_des.cross(proj_xb_des)).norm();
            xb_des = yb_des.cross(zb_des) / (yb_des.cross(zb_des)).norm();
          
            rotmat << xb_des(0), yb_des(0), zb_des(0), xb_des(1), yb_des(1), zb_des(1), xb_des(2), yb_des(2), zb_des(2);
            quat = rot2Quaternion(rotmat);
            return quat;
          }

        // Eigen::Vector3d computeVelocity(const Eigen::Vector3d &currPose, const Eigen::Vector3d &currVel, 
        //     const Eigen::Vector3d &targetPose, const Eigen::Vector3d &targetVel, double dt) {
            
        //     Eigen::Vector3d pos_error = targetPose - currPose;
        //     Eigen::Vector3d vel_error = targetVel - currVel;

        //     Eigen::Vector3d acc_des = kp_p_.asDiagonal() * pos_error + kd_p_.asDiagonal() * vel_error;
        //     Eigen::Vector3d vel_des = targetVel + acc_des * dt;  // 简单积分加速度得到速度指令

        //     if(vel_des.norm() > maxVel_){
        //         vel_des = (maxVel_ / vel_des.norm()) * vel_des;
        //     }
        //     return vel_des;
        // }

        // Eigen::Vector3d computeTakeOffVel(const Eigen::Vector3d &currPose, const double takeoff_height) {
        //     // 示例：梯形速度曲线生成目标速度
        //     Eigen::Vector3d targetVel;
        //     double acc_time = 1.2;   // 加速时间
        //     double current_height = currPose[2];

        //     double error_z = takeoff_height - current_height;
        //     if (error_z > 0) {
        //         // 根据误差动态调整目标速度
        //         targetVel.z() = min(maxVel_, error_z / acc_time);
        //     } else {
        //         targetVel.z() = 0;
        //     }
        //     return targetVel;
        // }

        double getDesiredThrust(){
            return thrust_des_;
        }

        void setParam(const Eigen::Vector3d &kp_p, const Eigen::Vector3d &kp_v, 
                      const Eigen::Vector3d &ki_v, const Eigen::Vector3d &kd_v,
                      const double &pos_error_max, const double &vel_error_max,
                      const double &vel_integral_max){
            kp_p_ = kp_p;
            kp_v_ = kp_v;
            ki_v_ = ki_v; 
            kd_v_ = kd_v;
            pos_error_max_ = pos_error_max;
            vel_error_max_ = vel_error_max; 
            vel_integral_max_ = vel_integral_max;
        }

        void printf_param(){
            cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>> cascade PID Parameter <<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

            cout <<"Kp_px : "<< kp_p_[0] << endl;
            cout <<"Kp_py : "<< kp_p_[1] << endl;
            cout <<"Kp_pz : "<< kp_p_[2] << endl;
            // cout <<"Kd_px : "<< kd_p_[0] << endl;
            // cout <<"Kd_py : "<< kd_p_[1] << endl;
            // cout <<"Kd_pz : "<< kd_p_[2] << endl;
            // cout <<"Ki_px : "<< ki_p_[0] << endl;
            // cout <<"Ki_py : "<< ki_p_[1] << endl;
            // cout <<"Ki_pz : "<< ki_p_[2] << endl;

            cout <<"Kp_vx : "<< kp_v_[0] << endl;
            cout <<"Kp_vy : "<< kp_v_[1] << endl;
            cout <<"Kp_vz : "<< kp_v_[2] << endl;
            cout <<"Kd_vx : "<< kd_v_[0] << endl;
            cout <<"Kd_vy : "<< kd_v_[1] << endl;
            cout <<"Kd_vz : "<< kd_v_[2] << endl;
            cout <<"Ki_vx : "<< ki_v_[0] << endl;
            cout <<"Ki_vy : "<< ki_v_[1] << endl;
            cout <<"Ki_vz : "<< ki_v_[2] << endl;

            cout <<"Limit:  " <<endl;
            cout <<"maxVel : "<< maxVel_ << endl;
            cout <<"macAcc : "<< maxAcc_ << endl;
            cout <<"pos_error_max : "<< pos_error_max_ << endl;
            // cout <<"pos_integral_max :  "<< pos_integral_max_ << endl;
            cout <<"vel_error_max : "<< vel_error_max_ << endl;
            cout <<"vel_integral_max :  "<< vel_integral_max_ << endl;
        }

    private:
        ros::NodeHandle nh_;

        Eigen::Vector3d kp_p_, ki_p_, kd_p_, kp_v_, ki_v_, kd_v_;
        
        double maxVel_, maxAcc_, thrust_des_;
        const double GRAVITY = 9.81;
        // double Kp_x_, Kp_y_, Kp_z_, Ki_x_, Ki_y_, Ki_z_, Kd_x_, Kd_y_, Kd_z_;
        Eigen::Vector3d integral_p_, pre_error_p_, integral_v_, pre_error_v_;
        double pos_error_max_, pos_integral_max_, vel_error_max_, vel_integral_max_;
};

#endif