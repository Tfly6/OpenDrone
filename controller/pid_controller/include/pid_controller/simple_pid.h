#ifndef SIMPLE_PID_H
#define SIMPLE_PID_H

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Dense>

#include "math_utils/math_utils.h"
using namespace std;

class simplePID {
    public:
        simplePID(void):nh_("~"){

            nh_.param<double>("Pos_pid/Kp_x", kp_[0], 1.0);
            nh_.param<double>("Pos_pid/Kp_y", kp_[1], 1.0);
            nh_.param<double>("Pos_pid/Kp_z" , kp_[2], 2.0);
            nh_.param<double>("Pos_pid/Kd_x", kd_[0], 0.5);
            nh_.param<double>("Pos_pid/Kd_y", kd_[1], 0.5);
            nh_.param<double>("Pos_pid/Kd_z" , kd_[2], 0.5);
            nh_.param<double>("Pos_pid/Ki_x", ki_[0], 0.2);
            nh_.param<double>("Pos_pid/Ki_y", ki_[1], 0.2);
            nh_.param<double>("Pos_pid/Ki_z" , ki_[2], 0.2);

            nh_.param<double>("Pos_pid/px_error_max", pos_error_max_[0], 0.6);
            nh_.param<double>("Pos_pid/py_error_max", pos_error_max_[1], 0.6);
            nh_.param<double>("Pos_pid/pz_error_max", pos_error_max_[2], 1.0);

            integral_ <<0, 0, 0;
            pre_error_ <<0, 0, 0;

        }

        Eigen::Vector3d compute(const Eigen::Vector3d &currPose, Eigen::Vector3d &targetPose, double dt){
            // 计算误差项
            Eigen::Vector3d pos_error = targetPose - currPose;

            // 误差项限幅
            for (int i=0; i<3; i++)
            {
                pos_error[i] = constrain_function(pos_error[i], pos_error_max_[i]);
            }

            integral_ += pos_error * dt;
            
            Eigen::Vector3d derivative = (pos_error - pre_error_) / dt;
            pre_error_ = pos_error;

            // PID
            Eigen::Vector3d pose_des;
            pose_des = kp_.asDiagonal() * pos_error + ki_.asDiagonal() * integral_ + kd_.asDiagonal() * derivative;
            

            return pose_des;
        }
        void printf_param(){
            cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>PID Parameter <<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;

            cout <<"Kp_x : "<< kp_[0] << endl;
            cout <<"Kp_y : "<< kp_[1] << endl;
            cout <<"Kp_z : "<< kp_[2] << endl;

            cout <<"Kd_x : "<< kd_[0] << endl;
            cout <<"Kd_y : "<< kd_[1] << endl;
            cout <<"Kd_z : "<< kd_[2] << endl;

            cout <<"Ki_x : "<< ki_[0] << endl;
            cout <<"Ki_y : "<< ki_[1] << endl;
            cout <<"Ki_z : "<< ki_[2] << endl;

            cout <<"Limit:  " <<endl;
            cout <<"px_error_max : "<< pos_error_max_[0] << endl;
            cout <<"py_error_max : "<< pos_error_max_[1] << endl;
            cout <<"pz_error_max :  "<< pos_error_max_[2] << endl;
        }

    private:
        ros::NodeHandle nh_;

        Eigen::Vector3d kp_, ki_, kd_;
        // double Kp_x_, Kp_y_, Kp_z_, Ki_x_, Ki_y_, Ki_z_, Kd_x_, Kd_y_, Kd_z_;
        Eigen::Vector3d integral_, pre_error_, pos_error_max_;
};

#endif