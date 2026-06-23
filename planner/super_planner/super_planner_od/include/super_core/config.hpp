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


#ifndef SUPER_PLANNER_CONFIG_HPP
#define SUPER_PLANNER_CONFIG_HPP

#include <rog_map/rog_map_core/config.hpp>
#include <traj_opt/config.hpp>
#include <utils/header/yaml_loader.hpp>

namespace super_planner {
    using namespace traj_opt;
    using std::cout;
    using std::endl;

    class Config {
    public:
        enum YawMode{
            YAW_TO_VEL = 1,
            YAW_TO_GOAL = 2
        };

        traj_opt::Config exp_traj_cfg, back_traj_cfg;

        // Bool Params
        bool visualization_en{true};
        bool detailed_log_en{false};
        bool backup_traj_en;
        bool use_fov_cut, print_log;
        bool goal_vel_en,goal_yaw_en;
        bool visual_process;
        bool frontend_in_known_free;

        double resolution;
        double planning_horizon;
        double receding_dis;
        double safe_corridor_line_max_length;
        // for fov cut
        double sensing_horizon;

        // Planning Params
        int obs_skip_num;
        double corridor_bound_dis, corridor_line_max_length;
        double replan_forward_dt;
        double sample_traj_dt;
        double robot_r;
        int iris_iter_num;

        int mpc_horizon{};

        double yaw_dot_max;
        // Yaw mode: 1 heading to velocity, 2 heading to goal
        int yaw_mode = YAW_TO_VEL;

        rog_map::vec_E<rog_map::Vec3i> seed_line_neighbour;


        Config() = default;
        Config(const std::string & cfg_path) {
            yaml_loader::YamlLoader loader(cfg_path);
            exp_traj_cfg = traj_opt::Config(cfg_path, "exp_traj");
            back_traj_cfg = traj_opt::Config(cfg_path, "backup_traj");
            loader.LoadParam("super_planner/print_log", print_log, false);
            loader.LoadParam("super_planner/detailed_log_en", detailed_log_en, false);
            loader.LoadParam("super_planner/visualization_en", visualization_en, false);
            loader.LoadParam("super_planner/backup_traj_en", backup_traj_en, false);
            loader.LoadParam("super_planner/goal_vel_en", goal_vel_en, false);
            loader.LoadParam("super_planner/goal_yaw_en", goal_yaw_en, false);
            loader.LoadParam("super_planner/visual_process", visual_process, false);
            loader.LoadParam("super_planner/use_fov_cut", use_fov_cut, false);
            loader.LoadParam("super_planner/frontend_in_known_free", frontend_in_known_free, false);
            loader.LoadParam("super_planner/safe_corridor_line_max_length", safe_corridor_line_max_length, 3.0);
            loader.LoadParam("super_planner/sensing_horizon", sensing_horizon, 3.0);
            loader.LoadParam("super_planner/obs_skip_num", obs_skip_num, 1);
            loader.LoadParam("super_planner/replan_forward_dt", replan_forward_dt, 0.3);
            loader.LoadParam("super_planner/corridor_bound_dis", corridor_bound_dis, 3.0);
            loader.LoadParam("super_planner/corridor_line_max_length", corridor_line_max_length, 3.0);
            loader.LoadParam("super_planner/planning_horizon", planning_horizon, 10.0);
            loader.LoadParam("super_planner/receding_dis", receding_dis, 5.0);
            loader.LoadParam("super_planner/robot_r", robot_r, 0.3);
            loader.LoadParam("super_planner/iris_iter_num", iris_iter_num, 1);
            loader.LoadParam("super_planner/yaw_mode", yaw_mode, 1);
            loader.LoadParam("super_planner/mpc_horizon", mpc_horizon, 1);
            loader.LoadParam("super_planner/yaw_dot_max", yaw_dot_max, 3.14);

            loader.LoadParam("rog_map/resolution", resolution, 0.01, true);

            sample_traj_dt = resolution / exp_traj_cfg.max_vel;

            int step = ceil(robot_r / resolution);
            for (int x = -step; x <= step; x++) {
                for (int y = -step; y <= step; y++) {
                    for (int z = -step; z <= step; z++) {
                        if (x * x + y * y + z * z <= step * step) {
                            seed_line_neighbour.push_back({x, y, z});
                        }
                    }
                }
            }
            std::sort(seed_line_neighbour.begin(), seed_line_neighbour.end(),
                      [](const auto& a, const auto& b) {
                          return a[0] * a[0] + a[1] * a[1] + a[2] * a[2] < b[0] * b[0] + b[1] * b[1] + b[2] * b[2];
                      });
        }


    };
}

#endif
