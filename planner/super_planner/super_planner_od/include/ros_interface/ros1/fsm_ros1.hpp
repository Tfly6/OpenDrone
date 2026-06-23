/**
* This file is part of SUPER
*
* Copyright 2025 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/SUPER>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* SUPER is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* SUPER is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with SUPER. If not, see <http://www.gnu.org/licenses/>.
*/


#ifdef USE_ROS1

#ifndef SRC_FSM_ROS1_HPP
#define SRC_FSM_ROS1_HPP

#include "fsm/fsm.h"

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "quadrotor_msgs/PolynomialTrajectory.h"


namespace fsm {
    class FsmRos1 : public Fsm {
        ros::NodeHandle nh_;
        ros::Subscriber goal_sub_;
        ros::Publisher cmd_pub, mpc_cmd_pub_, path_pub_;
        ros::Timer execution_timer_, replan_timer_, cmd_timer_;
        quadrotor_msgs::PositionCommand pid_cmd_;
        rog_map::ROGMapROS::Ptr map_ptr_;
        quadrotor_msgs::PositionCommand latest_cmd;
        nav_msgs::Path path;

        vector<quadrotor_msgs::PositionCommand> cmd_logs_;

        void resetVisualizedPath() override {
            path.poses.clear();
        }

        void publishCurPoseToPath() override {
            path.header.frame_id = "world";
            path.header.stamp = ros::Time::now();
            geometry_msgs::PoseStamped pose;
            pose.header = path.header;
            pose.pose.position.x = robot_state_.p(0);
            pose.pose.position.y = robot_state_.p(1);
            pose.pose.position.z = robot_state_.p(2);
            pose.pose.orientation.x = robot_state_.q.x();
            pose.pose.orientation.y = robot_state_.q.y();
            pose.pose.orientation.z = robot_state_.q.z();
            pose.pose.orientation.w = robot_state_.q.w();
            path.poses.push_back(pose);
            path_pub_.publish(path);
        }

        void publishPolyTraj() override {
            quadrotor_msgs::PolynomialTrajectory cmd_traj;
            getCommittedTrajectory(cmd_traj);
            mpc_cmd_pub_.publish(cmd_traj);
        }

        void getOneHeartBeatMsg(quadrotor_msgs::PolynomialTrajectory &heartbeat, bool &traj_finish) {
            heartbeat.type = quadrotor_msgs::PolynomialTrajectory::HEART_BEAT;
            heartbeat.header.stamp = ros::Time::now();
            heartbeat.header.frame_id = "world";
            double swt;
            planner_ptr_->getOneHeartbeatTime(swt, traj_finish);
            heartbeat.start_WT_pos = ros::Time(swt);
        }

        void getCommittedTrajectory(quadrotor_msgs::PolynomialTrajectory &cmd_traj) {
            cmd_traj.header.stamp = ros::Time::now();
            cmd_traj.header.frame_id = "world";
            cmd_traj.type = quadrotor_msgs::PolynomialTrajectory::POSITION_TRAJ |
                            quadrotor_msgs::PolynomialTrajectory::HEART_BEAT;
            planner_ptr_->lockCommittedTraj();
            const Trajectory pos_traj = planner_ptr_->getCommittedPositionTrajectory();
            const Trajectory yaw_traj = planner_ptr_->getCommittedYawTrajectory();
            planner_ptr_->unlockCommittedTraj();

            cmd_traj.start_WT_pos = ros::Time(pos_traj.start_WT);

            cmd_traj.piece_num_pos = pos_traj.getPieceNum();
            cmd_traj.order_pos = 7;
            cmd_traj.time_pos.resize(pos_traj.getPieceNum());
            cmd_traj.coef_pos_x.resize(cmd_traj.piece_num_pos * (cmd_traj.order_pos + 1));
            cmd_traj.coef_pos_y.resize(cmd_traj.piece_num_pos * (cmd_traj.order_pos + 1));
            cmd_traj.coef_pos_z.resize(cmd_traj.piece_num_pos * (cmd_traj.order_pos + 1));
            cmd_traj.start_WT_pos = ros::Time(pos_traj.start_WT);

            if (!yaw_traj.empty()) {
                cmd_traj.type = cmd_traj.type |
                                quadrotor_msgs::PolynomialTrajectory::YAW_TRAJ;
                cmd_traj.piece_num_yaw = yaw_traj.getPieceNum();
                cmd_traj.order_yaw = 7;
                double col_size = cmd_traj.order_yaw + 1;
                cmd_traj.coef_yaw.resize(cmd_traj.piece_num_yaw * col_size);
                cmd_traj.time_yaw.resize(cmd_traj.piece_num_yaw);
                for (int i = 0; i < cmd_traj.piece_num_yaw; i++) {
                    Eigen::VectorXd yaw_coef = yaw_traj[i].getCoeffMat().row(0);
                    Eigen::Map<Eigen::VectorXd>(&cmd_traj.coef_yaw[col_size * i], col_size) = yaw_coef;
                    cmd_traj.time_yaw[i] = yaw_traj[i].getDuration();
                }
                cmd_traj.start_WT_yaw = ros::Time(yaw_traj.start_WT);
            }

            for (int i = 0; i < cmd_traj.piece_num_pos; i++) {
                Eigen::Matrix<double, 3, 8> coef = pos_traj[i].getCoeffMat();
                Eigen::Map<Eigen::VectorXd>(&cmd_traj.coef_pos_x[8 * i], 8) = coef.row(0);
                Eigen::Map<Eigen::VectorXd>(&cmd_traj.coef_pos_y[8 * i], 8) = coef.row(1);
                Eigen::Map<Eigen::VectorXd>(&cmd_traj.coef_pos_z[8 * i], 8) = coef.row(2);
                cmd_traj.time_pos[i] = pos_traj[i].getDuration();
            }
        }

        void getOnePositionCommand(quadrotor_msgs::PositionCommand &pos_cmd, bool &traj_finish) {
            pos_cmd.trajectory_flag = 0;
            StatePVAJ pvaj;
            double yaw, yaw_dot;
            bool on_backup_traj;
            planner_ptr_->getOneCommandFromTraj(pvaj, yaw, yaw_dot, on_backup_traj, traj_finish);
            pos_cmd.header.stamp = ros::Time::now();
            pos_cmd.header.frame_id = "world";
            pos_cmd.position.x = pvaj(0, 0);
            pos_cmd.position.y = pvaj(1, 0);
            pos_cmd.position.z = pvaj(2, 0);
            pos_cmd.velocity.x = pvaj(0, 1);
            pos_cmd.velocity.y = pvaj(1, 1);
            pos_cmd.velocity.z = pvaj(2, 1);
            pos_cmd.acceleration.x = pvaj(0, 2);
            pos_cmd.acceleration.y = pvaj(1, 2);
            pos_cmd.acceleration.z = pvaj(2, 2);
            pos_cmd.jerk.x = pvaj(0, 3);
            pos_cmd.jerk.y = pvaj(1, 3);
            pos_cmd.jerk.z = pvaj(2, 3);
            pos_cmd.yaw = yaw;
            pos_cmd.yaw_dot = yaw_dot;
            pos_cmd.trajectory_flag = on_backup_traj ? 2 : 1;
            Vec3f rpy, omg;
            double aT;
            geometry_utils::convertFlatOutputToAttAndOmg(pvaj.col(0), pvaj.col(1), pvaj.col(2), pvaj.col(3), yaw,
                                                         yaw_dot, rpy, omg, aT);
            pos_cmd.attitude.x = rpy(0);
            pos_cmd.attitude.y = rpy(1);
            pos_cmd.attitude.z = rpy(2);
            pos_cmd.angular_velocity.x = omg(0);
            pos_cmd.angular_velocity.y = omg(1);
            pos_cmd.angular_velocity.z = omg(2);
            pos_cmd.thrust.z = aT;
            latest_cmd = pos_cmd;
            cmd_logs_.push_back(latest_cmd);
        }

    public:
        FsmRos1() = default;

        ~FsmRos1(){
            ros::shutdown();
            saveReplanLogToFile("super_latest_log");
            exit(0);
        };

        typedef std::shared_ptr<FsmRos1> Ptr;

        void saveReplanLogToFile(const string &name = "") {
            // run statistic
            double total_length{0.0};
            int total_replan_num{0};
            double average_compt_t{0.0};
            Vec3f cur_p{0, 0, 0};
            for (auto rp: replan_logs_) {
                if (rp.getRetCode() > 0) {
                    if (cur_p.norm() < 1e-6) {
                        cur_p = rp.getRobotP();
                    } else {
                        total_length += (rp.getRobotP() - cur_p).norm();
                        cur_p = rp.getRobotP();
                    }
                    total_replan_num++;
                    average_compt_t += rp.getTotalCompT();
                }
            }


            fmt::print("Total replan num: {}, total length: {}, average computation time: {} ms\n",
                       total_replan_num, total_length, average_compt_t / (total_replan_num==0?1:total_replan_num) * 1000);


            const std::string save_path = name.empty()
                                          ? LOG_FILE_DIR(
                                                  "replan_logs/" + BinaryFileHandler<int>::getCurrentTimeStr() + ".bin")
                                          : LOG_FILE_DIR("replan_logs/" + name + ".bin");
            const std::string csv_path = name.empty()
                                         ? LOG_FILE_DIR(
                                                 "cmd_logs/" + BinaryFileHandler<int>::getCurrentTimeStr() + ".csv")
                                         : LOG_FILE_DIR("cmd_logs/" + name + ".csv");
            BinaryFileHandler<vector<LogOneReplan>>::save(save_path, replan_logs_);

            std::ofstream csv_writer;
            csv_writer.open(csv_path, std::ios::out | std::ios::trunc);
            csv_writer
                    << "time,posi_x,posi_y,posi_z,vel_x,vel_y,vel_z,acc_x,acc_y,acc_z,jerk_x,jerk_y,jerk_z,yaw,yaw_rate,backup"
                    << std::endl;
            csv_writer<<std::fixed<<std::setprecision(15);
            for (const auto &cmd: cmd_logs_) {
                csv_writer << cmd.header.stamp.toSec() - system_start_time_ << "," << cmd.position.x << "," << cmd.position.y << ","
                           << cmd.position.z << ","
                           << cmd.velocity.x << "," << cmd.velocity.y << "," << cmd.velocity.z << ","
                           << cmd.acceleration.x << "," << cmd.acceleration.y << "," << cmd.acceleration.z << ","
                           << cmd.jerk.x << "," << cmd.jerk.y << "," << cmd.jerk.z << ","
                           << cmd.yaw << "," << cmd.yaw_dot << "," << static_cast<int>(cmd.trajectory_flag)
                           << std::endl;
            }
            csv_writer.close();
        }

        bool getPoseFromTraj(super_utils::Pose &pose) {
            if (machine_state_ != FOLLOW_TRAJ) {
                cout << YELLOW << "[Fsm] Not in FOLLOW_TRAJ state, can't get pose from traj." << RESET << endl;
                return false;
            }
            getOnePositionCommand(pid_cmd_, traj_finish_);
            if (traj_finish_) {
                cout << GREEN << " -- [Fsm] Traj finish." << RESET << endl;
                if (closeToGoal(0.1)) {
                    ChangeState("getPoseFromTraj", WAIT_GOAL);
                } else {
                    ChangeState("getPoseFromTraj", GENERATE_TRAJ);
                }
            }
            pose.first = Vec3f{pid_cmd_.position.x, pid_cmd_.position.y, pid_cmd_.position.z};
            pose.second = eulerToQuaternion(pid_cmd_.attitude.x, pid_cmd_.attitude.y, pid_cmd_.attitude.z);


            /// for checking the trajectory continuty
            static double max_delta_v{0.0};
            static double last_v = pid_cmd_.vel_norm;
            double delta_v = std::abs(pid_cmd_.vel_norm - last_v);
            last_v = pid_cmd_.vel_norm;
            if (delta_v > max_delta_v) {
                max_delta_v = delta_v;
            }
            fmt::print(" -- [Fsm] Cur vel: {}, delta_v: {}, max_delta_v: {}\n", pid_cmd_.vel_norm, delta_v,
                       max_delta_v);
            cmd_logs_.push_back(latest_cmd);
            return true;
        }

        void goalCallback(const geometry_msgs::PoseStampedConstPtr &msg) {
            super_utils::Vec3f goal_p = Vec3f{msg->pose.position.x, msg->pose.position.y, msg->pose.position.z};
            super_utils::Quatf goal_q = super_utils::Quatf{msg->pose.orientation.w, msg->pose.orientation.x,
                                                           msg->pose.orientation.y, msg->pose.orientation.z};
            setGoalPosiAndYaw(goal_p, goal_q);
        }

        void init(const ros::NodeHandle &nh, const std::string &cfg_path) {
            // 初始化参数读取
            nh_ = nh;
            cfg_ = Config(cfg_path);
            map_ptr_ = std::make_shared<rog_map::ROGMapROS>(nh, cfg_path);
            // 初始化Planner
            ros_ptr_ = std::make_shared<ros_interface::Ros1Interface>(nh_);
            planner_ptr_ = std::make_shared<SuperPlanner>(cfg_path, ros_ptr_, map_ptr_);
            cmd_pub = nh_.advertise<quadrotor_msgs::PositionCommand>(cfg_.cmd_topic, 10);
            mpc_cmd_pub_ = nh_.advertise<quadrotor_msgs::PolynomialTrajectory>(cfg_.mpc_cmd_topic, 10);
            path_pub_ = nh_.advertise<nav_msgs::Path>("fsm/path", 100);

            int cmd_cnt = 0;

            if (cfg_.click_goal_en) {
                goal_sub_ = nh_.subscribe(cfg_.click_goal_topic, 1, &FsmRos1::goalCallback, this);
                cout << YELLOW << " -- [Fsm] CLICKGOAL ENABLE." << RESET << endl;
                cmd_cnt++;
            }

            if (cmd_cnt != 1) {
                cout << YELLOW << " -- [Fsm] CMD INPUT ERROR." << RESET << endl;
                exit(0);
            }

            if (cfg_.timer_en) {
                execution_timer_ = nh_.createTimer(ros::Duration(0.01), &FsmRos1::mainFsmTimerCallback, this); // 100Hz
                cmd_timer_ = nh_.createTimer(ros::Duration(0.01), &FsmRos1::pubCmdTimerCallback, this); // 100Hz
                replan_timer_ = nh_.createTimer(ros::Duration(1.0 / cfg_.replan_rate), &FsmRos1::replanTimerCallback,
                                                this); // 10Hz
            }

            write_time_.open(DEBUG_FILE_DIR("time_consuming.csv"), std::ios::out | std::ios::trunc);
            log_module_time.resize(9);
            for (int i = 0; i < 9; i++) {
                write_time_ << log_time_str[i];
                if (i != 8) {
                    write_time_ << ",";
                }
            }
            write_time_ << endl;
            machine_state_ = INIT;
            system_start_time_ = ros_ptr_->getSimTime();

            pid_cmd_.kx[0] = 5.7;
            pid_cmd_.kx[1] = 5.7;
            pid_cmd_.kx[2] = 4.2;

            pid_cmd_.kv[0] = 3.4;
            pid_cmd_.kv[1] = 3.4;
            pid_cmd_.kv[2] = 4.0;
        }

        void pubCmdTimerCallback(const ros::TimerEvent &event) {
            if (stop) {
                return;
            }
            if (machine_state_ != FOLLOW_TRAJ && machine_state_ != EMER_STOP) {
                return;
            }


            quadrotor_msgs::PolynomialTrajectory heartbeat;
            getOneHeartBeatMsg(heartbeat, traj_finish_);
            getOnePositionCommand(pid_cmd_, traj_finish_);
            mpc_cmd_pub_.publish(heartbeat);
            cmd_pub.publish(pid_cmd_);
            if (traj_finish_) {
                cout << GREEN << " -- [Fsm] Traj finish." << RESET << endl;
                if (closeToGoal(0.1)) {
                    ChangeState("PubCmdCallback", WAIT_GOAL);
                } else {
                    ChangeState("PubCmdCallback", GENERATE_TRAJ);
                }
            }
        }

        void replanTimerCallback(const ros::TimerEvent &event) {
            callReplanOnce();
        }

        void mainFsmTimerCallback(const ros::TimerEvent &event) {
            callMainFsmOnce();
        }

    };
}

#endif //SRC_FSM_ROS1_HPP

#endif