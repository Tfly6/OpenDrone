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


#pragma once

#include <queue>
#include <memory>
#include <fstream>
#include <fmt/color.h>
#include <cereal/archives/binary_file_handler.hpp>
#include <fsm/config.hpp>
#include <super_core/super_planner.h>


#ifndef LOG_FILE_DIR
#define LOG_FILE_DIR(name) (string(string(ROOT_DIR) + "log/"+name))
#endif

namespace fsm {
    class Fsm {
    protected:
        bool stop{false};


        vector<string> log_time_str{
                "TIME_STAMPE", "EPX_TRAJ_FRONTEND",
                "EXP_TRAJ_OPT", "GENERATE_EXP_TRAJ",
                "BACK_TRAJ_FRONTEND", "BACK_TRAJ_OPT",
                "GENERATE_BACK_TRAJ", "TOTAL_REPLAN", "VISUALIZATION"
        };
        Config cfg_;
        // map, checker, planner
        super_planner::SuperPlanner::Ptr planner_ptr_;
        ros_interface::RosInterface::Ptr ros_ptr_;

        std::ofstream write_time_;
        vector<double> log_module_time;
        double yaw_{0}, yaw_dot_{0};

        rog_map::RobotState robot_state_;

        // params
        bool started_{false}, plan_from_rest_{false};

        struct GoalInfo {
            bool new_goal;
            Vec3f goal_p;
            double goal_yaw;
        } gi_;

        Eigen::Vector3d auto_pilot_vel_w_;

        // execution states
        enum MACHINE_STATE {
            INIT = 0,
            WAIT_GOAL,
            YAWING,
            GENERATE_TRAJ,
            FOLLOW_TRAJ,
            EMER_STOP
        };

        vector<string> MACHINE_STATE_STR{
                "INIT",
                "WAIT_GOAL",
                "YAWING",
                "GENERATE_TRAJ",
                "FOLLOW_TRAJ", "EMER_STOP"
        };


        MACHINE_STATE machine_state_{INIT};


    public:
        Fsm() = default;
        ~Fsm();

        void updateROGMap(const rog_map::PointCloud &cloud, const super_utils::Pose &pose) {
            planner_ptr_->updateROGMap(cloud, pose);
        }

        void callPlanOnce(const Vec3f &goal) {
            TimeConsuming tc("Call replan once time", true);
            fmt::print(" -- [Fsm] Call plan once, cur state {}.\n", MACHINE_STATE_STR[machine_state_]);
            // check current state;
            Quatf q(NAN, NAN, NAN, NAN);
            setGoalPosiAndYaw(goal, q);

            callMainFsmOnce();

            if (machine_state_ == FOLLOW_TRAJ) {
                callReplanOnce();
            }

            // save on log
            replan_logs_.push_back(planner_ptr_->getLatestReplanLog());
            fmt::print(fmt::fg(fmt::color::green), " -- Replan ID: {}, ret code: {}\n",
                       replan_logs_.size() - 1, replan_logs_.back().getRetCode());
        }

        Eigen::Quaterniond eulerToQuaternion(double roll, double pitch, double yaw) {
            double half_roll = roll * 0.5;
            double half_pitch = pitch * 0.5;
            double half_yaw = yaw * 0.5;

            double sin_r = std::sin(half_roll);
            double cos_r = std::cos(half_roll);
            double sin_p = std::sin(half_pitch);
            double cos_p = std::cos(half_pitch);
            double sin_y = std::sin(half_yaw);
            double cos_y = std::cos(half_yaw);

            // 计算四元数分量
            Eigen::Quaterniond q;
            q.w() = cos_r * cos_p * cos_y + sin_r * sin_p * sin_y;
            q.x() = sin_r * cos_p * cos_y - cos_r * sin_p * sin_y;
            q.y() = cos_r * sin_p * cos_y + sin_r * cos_p * sin_y;
            q.z() = cos_r * cos_p * sin_y - sin_r * sin_p * cos_y;

            return q;
        }

    protected:
        vector<LogOneReplan> replan_logs_;
        /* Callback functions */
        bool finish_plan = false;
        double system_start_time_;

        bool traj_finish_{false};

        void WriteTimeToLog();

        void callReplanOnce();

        void callMainFsmOnce();

        bool closeToGoal(const double &thresh_dis);

        void setGoalPosiAndYaw(const Vec3f &p, const Quatf &q);

        void ChangeState(const string &call_func, const MACHINE_STATE &new_state);

        virtual void publishPolyTraj() = 0;

        virtual void publishCurPoseToPath() = 0;

        virtual void resetVisualizedPath() = 0;
    };
}
