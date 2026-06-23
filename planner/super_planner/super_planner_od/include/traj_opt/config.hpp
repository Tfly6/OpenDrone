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

#include <string>
#include <utils/geometry/quadrotor_flatness.hpp>
#include <utils/header/yaml_loader.hpp>
#define DEBUG_FILE_DIR(name) (string(string(ROOT_DIR) + "log/"+(name)))

namespace traj_opt {
    using std::string;
    using std::vector;

    enum PosConstrainType {
        WAYPOINT = 1,
        CORRIDOR = 2,
    };

    class Config {
    public:
        bool uniform_time_en{false};

        flatness::FlatnessMap quadrotot_flatness;

        bool print_optimizer_log{false};

        /// Param for flatness
        double mass, dh, dv, grav, cp, v_eps;

        // if save the optimization problem to log
        bool save_log_en{false};

        int pos_constraint_type{CORRIDOR};
        // Set to true for only min time.
        bool block_energy_cost{false};
        // Limit conditions.
        double max_vel{0}, max_acc{0}, max_jerk{0}, max_omg{0}, max_acc_thr{0}, min_acc_thr{0};
        // Penalty cost.
        double penna_scale{-1}, penna_vel{0}, penna_acc{0}, penna_jerk{0}, penna_omg{0}, penna_thr{0};
        // penna_t; penna_pos only for corridor based method.
        double penna_t{0}, penna_pos{0}, penna_attract{0};
        // penna_ts only for backupTraj;
        double penna_ts{0};
        // for backup traj piece num
        int piece_num{0};

        double penna_margin{0.05};

        double smooth_eps{0};
        int integral_reso{0};
        double opt_accuracy{0};

        Config() = default;

        Config(const std::string & cfg_path, string ns) {
            yaml_loader::YamlLoader loader(cfg_path);
            if (ns.empty()) {
                ns = "/";
            }
            else {
                ns = "/" + ns + "/";
            }

            loader.LoadParam("traj_opt/switch/print_optimizer_log", print_optimizer_log, false);
            /// Load Param for Flatness
            loader.LoadParam("traj_opt/flatness/mass", mass, 1.0);
            loader.LoadParam("traj_opt/flatness/dh", dh, 0.7);
            loader.LoadParam("traj_opt/flatness/dv", dv, 0.8);
            loader.LoadParam("traj_opt/flatness/grav", grav, 1.0);
            loader.LoadParam("traj_opt/flatness/cp", cp, 0.01);
            loader.LoadParam("traj_opt/flatness/v_eps", v_eps, 0.0001);

            loader.LoadParam("traj_opt/switch/save_log_en", save_log_en, false);
            loader.LoadParam("traj_opt" + ns + "pos_constraint_type", pos_constraint_type, 2);
            loader.LoadParam("traj_opt" + ns + "piece_num", piece_num, 1);
            loader.LoadParam("traj_opt" + ns + "uniform_time_en", uniform_time_en, false);
            loader.LoadParam("traj_opt" + ns + "block_energy_cost", block_energy_cost, false);
            loader.LoadParam("traj_opt" + ns + "opt_accuracy", opt_accuracy, 1.0e-5);
            loader.LoadParam("traj_opt" + ns + "integral_reso", integral_reso, 10);
            loader.LoadParam("traj_opt" + ns + "smooth_eps", smooth_eps, 0.01);
            loader.LoadParam("traj_opt/boundary/max_vel", max_vel, -1.0);
            loader.LoadParam("traj_opt/boundary/max_acc", max_acc, -1.0);
            loader.LoadParam("traj_opt/boundary/max_jerk", max_jerk, -1.0);
            loader.LoadParam("traj_opt/boundary/max_omg", max_omg, -1.0);
            loader.LoadParam("traj_opt/boundary/max_acc_thr", max_acc_thr, -1.0);
            loader.LoadParam("traj_opt/boundary/min_acc_thr", min_acc_thr, -1.0);
            loader.LoadParam("traj_opt/boundary/penna_margin", penna_margin, 0.05);

            loader.LoadParam("traj_opt" + ns + "penna_scale", penna_scale, -1.0);
            loader.LoadParam("traj_opt" + ns + "penna_t", penna_t, -1.0);
            loader.LoadParam("traj_opt" + ns + "penna_ts", penna_ts, -1.0);
            loader.LoadParam("traj_opt" + ns + "penna_pos", penna_pos, -1.0);
            loader.LoadParam("traj_opt" + ns + "penna_vel", penna_vel, -1.0);
            loader.LoadParam("traj_opt" + ns + "penna_acc", penna_acc, -1.0);
            loader.LoadParam("traj_opt" + ns + "penna_jerk", penna_jerk, -1.0);
            loader.LoadParam("traj_opt" + ns + "penna_attract", penna_attract, -1.0);
            loader.LoadParam("traj_opt" + ns + "penna_omg", penna_omg, -1.0);
            loader.LoadParam("traj_opt" + ns + "penna_thr", penna_thr, -1.0);

            if (penna_scale > 0) {
                penna_t = penna_t * penna_scale;
                penna_ts = penna_ts * penna_scale;
                penna_pos = penna_pos * penna_scale;
                penna_vel = penna_vel * penna_scale;
                penna_acc = penna_acc * penna_scale;
                penna_jerk = penna_jerk * penna_scale;
                penna_attract = penna_attract * penna_scale;
                penna_omg = penna_omg * penna_scale;
                penna_thr = penna_thr * penna_scale;
            }

            quadrotot_flatness.reset(mass, grav, dh, dv, cp, v_eps);
        }
    };
}
