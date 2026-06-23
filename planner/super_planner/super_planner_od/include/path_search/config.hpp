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

#include "vector"
#include "Eigen/Dense"
#include "string"
#include "utils/header/type_utils.hpp"
#include "utils/header/yaml_loader.hpp"

namespace path_search {
    using super_utils::Vec3i;
    using std::cout;
    using std::endl;
    using std::string;
    using std::vector;

    class PathSearchConfig {
    public:
        Vec3i map_voxel_num, map_size_i;
        bool visual_process;
        bool debug_visualization_en;
        bool allow_diag{false};
        int heu_type{0};

        PathSearchConfig() {};

        PathSearchConfig(const string & cfg_path, const string name_space = "astar") {
            yaml_loader::YamlLoader loader(cfg_path);
            vector<int> vox_;
            loader.LoadParam(name_space + "/map_voxel_num", vox_, vox_);
            loader.LoadParam(name_space + "/allow_diag", allow_diag, false);
            loader.LoadParam(name_space + "/debug_visualization_en", debug_visualization_en, false);
            loader.LoadParam(name_space + "/heu_type", heu_type, 0);
            loader.LoadParam(name_space + "/visual_process", visual_process, false);
            map_voxel_num = Vec3i(vox_[0], vox_[1], vox_[2]);
            map_size_i = map_voxel_num / 2;
            map_voxel_num = map_size_i * 2 + Vec3i::Constant(1);
        }

    };
}
