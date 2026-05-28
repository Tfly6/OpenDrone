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

#include "traj_opt/yaw_traj_opt.h"
#include <utils/optimization/polynomial_interpolation.h>
using namespace geometry_utils;
namespace traj_opt {
    using namespace color_text;
    void YawTrajOpt::getYawTimeAllocation(const double &duration, VecDf &times) const {
        double interp_dt = M_PI / yaw_dot_max_;
        if (duration < interp_dt * 2) {
            /// if the duration less than 2 interp, then no waypoint need.
            times.resize(1);
            times[0] = duration;
        } else {
            /// use ceil to make the seg num non-zero and, make sure the last seg
            /// have time to turn to the goal yaw.
            int interp_num = ceil((duration - 2 * interp_dt) / interp_dt);
            double interp_t = (duration - 2 * interp_dt) / (interp_num);
            times.resize(2 + interp_num);
            for (int i = 0; i < interp_num; i++) {
                times(i + 1) = interp_t;
            }
            times(times.size() - 1) = interp_dt;
            times(0) = interp_dt;
        }
        if (times.size() == 3 && times(1) < times(0) / 3) {
            times.resize(2);
            times.setConstant(duration / 2);
        }
    }

    void YawTrajOpt::getYawWaypointAllocation(const Vec4f &init_state, Vec4f &goal_state, VecDf &way_pts, VecDf &times,
                                              const Trajectory &pos_traj) {
        double eval_t = 0;
        vec_Vec3f debug1, debug2;
        double last_yaw = init_state(0);
        way_pts.resize(times.size() - 1);
        const double pos_traj_duration = pos_traj.getTotalDuration();
        for (long int i = 0; i < times.size() - 1; i++) {
            eval_t += times(i);
            double cur_yaw;
            Vec3f pt_i = pos_traj.getPos(eval_t);
            Vec3f pt_g;
            if (eval_t + 0.5 >= pos_traj_duration) {
                pt_g = pos_traj.getPos(pos_traj_duration);
                pt_i = pos_traj.getPos(pos_traj_duration - 0.5 > 0 ? pos_traj_duration - 0.5 : 0);
            } else {
                pt_g = pos_traj.getPos(eval_t + 0.5);
            }


            Vec3f dir = pt_g - pt_i;
            if (dir.norm() > 0.1) {
                cur_yaw = atan2(dir.y(), dir.x());
                normalizeNextYaw(last_yaw, cur_yaw);
            } else {
//                    print(fg(color::indian_red),
//                          " -- [SUPER] Yaw planning failed, the goal yaw is too close to the current yaw.\n");
                cur_yaw = last_yaw;
            }
            way_pts(i) = cur_yaw;
            last_yaw = cur_yaw;
        }
        if (way_pts.size() == 0) {
            geometry_utils::normalizeNextYaw(init_state[0], goal_state[0]);
        } else {
            geometry_utils::normalizeNextYaw(way_pts(way_pts.size() - 1), goal_state[0]);
        }
//            print("Remain dis = {}.\n", (cur_drone_state_.position - gi_.mission_waypoints.back()).norm());
//            print("Yaw way_pts init = {}\n", init_state(0));
//            for (long unsigned int i = 0; i < way_pts.size(); i++) {
//                print("Yaw way_pts[{}] = {}\n", i, way_pts[i]);
//            }
//            print("Yaw way_pts goal = {}\n", goal_state(0));
//            cout << times.transpose() << endl;
    }

    YawTrajOpt::YawTrajOpt(const double &_yaw_dot_max) : yaw_dot_max_(_yaw_dot_max) {
    }

    bool YawTrajOpt::optimize(const Vec4f &istate_in,
                              const Vec4f &gstate_in,
                              const Trajectory &pos_traj,
                              Trajectory &out_traj,
                              const int & order,
                              const bool &free_start,
                              const bool &free_goal) {
        free_goal_ = free_goal;
        Vec4f init_state = istate_in;
        Vec4f goal_state = gstate_in;
        double pos_traj_dur = pos_traj.getTotalDuration();
        if (free_start) {
            Vec3f pt_i = pos_traj.getPos(0);

            double t_g = pos_traj_dur > 0.5 > 0 ? 0.5 : pos_traj_dur;
            Vec3f pt_g = pos_traj.getPos(t_g);
            Vec3f dir = pt_g - pt_i;
            while (dir.norm() < 0.5 && t_g < pos_traj_dur) {
                t_g += 0.1;
                pt_g = pos_traj.getPos(t_g);
                dir = pt_g - pt_i;
            }
            init_state(0) = atan2(dir.y(), dir.x());
        }
        if (free_goal_) {
            Vec3f pt_g = pos_traj.getPos(pos_traj_dur);
            double t_i = pos_traj_dur - 0.5 > 0 ? pos_traj_dur - 0.5 : 0;
            Vec3f pt_i = pos_traj.getPos(t_i);
            Vec3f dir = pt_g - pt_i;
            while (dir.norm() < 0.5 && t_i > 0) {
                t_i -= 0.1;
                pt_i = pos_traj.getPos(t_i);
                dir = pt_g - pt_i;
            }
            goal_state(0) = atan2(dir.y(), dir.x());
        }

        VecDf times;
        getYawTimeAllocation(pos_traj_dur, times);
        VecDf way_pts;
        getYawWaypointAllocation(init_state, goal_state, way_pts, times, pos_traj);
        Trajectory yaw_traj;
        switch (order) {
            case 3: {
                const super_utils::Vec2f init_state3 = init_state.head(2);
                const super_utils::Vec2f goal_state3 = goal_state.head(2);
                yaw_traj = poly_interpo::minimumAccInterpolation<1>(init_state3,
                                                                    goal_state3,
                                                                    way_pts,
                                                                    times);
                break;
            }
            case 5: {
                Vec3f init_state3 = init_state.head(3);
                Vec3f goal_state3 = goal_state.head(3);
                yaw_traj = poly_interpo::minimumJerkInterpolation<1>(init_state3,
                                                                     goal_state3,
                                                                     way_pts,
                                                                     times);
                break;
            }
            case 7: {
                yaw_traj = poly_interpo::minimumSnapInterpolation<1>(init_state,
                                                                     goal_state,
                                                                     way_pts,
                                                                     times);
                break;
            }
            default: {
                cout << "Unsupported order for yaw trajectory optimization." << endl;
                return false;
            }
        }

//            for (double eval_t = 0; eval_t < yaw_traj.getTotalDuration(); eval_t += 0.01) {
//                cout << yaw_traj.getPos(eval_t)[0] << " ";
//            }
//            cout << ";" << endl;
//            for (double eval_t = 0; eval_t < yaw_traj.getTotalDuration(); eval_t += 0.01) {
//                cout << yaw_traj.getVel(eval_t)[0] << " ";
//            }
//            cout << endl;
//
//            yaw_traj.printProfile();
        double max_yaw_rate = yaw_traj.getMaxVelRate();
        if (max_yaw_rate > yaw_dot_max_ + 2.0) {
            cout << YELLOW << " Yaw rate too large, " << max_yaw_rate << RESET << endl;
//                return false;
        }
        out_traj = yaw_traj;
        out_traj.start_WT = pos_traj.start_WT;
        return true;
    }
}