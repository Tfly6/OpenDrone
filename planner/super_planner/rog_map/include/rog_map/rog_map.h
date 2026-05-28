/**
* This file is part of ROG-Map
*
* Copyright 2024 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/ROG-Map>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* ROG-Map is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ROG-Map is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with ROG-Map. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <rog_map/prob_map.h>
#include <rog_map/rog_map_core/common_lib.hpp>
#include <super_utils/type_utils.hpp>
#include <fmt/color.h>

namespace rog_map {
    using namespace std;
    using super_utils::vec_Vec3i;
    using super_utils::RobotState;

    typedef pcl::PointXYZI PointType;
    typedef pcl::PointCloud<PointType> PointCloudXYZIN;

    class ROGMap : public ProbMap {
        const bool IS = true;
        const bool NOT = false;

        virtual const double getSystemWalltimeNow() = 0;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        explicit ROGMap() = default;

        void init();

        ~ROGMap() override = default;

        rog_map::Config getMapConfig() const {
            return cfg_;
        }


        bool isLineFree(const Vec3f& start_pt, const Vec3f& end_pt,
                        const double& max_dis = 999999,
                        const vec_Vec3i& neighbor_list = vec_Vec3i{}) const;

        bool isLineFree(const Vec3f& start_pt, const Vec3f& end_pt,
                        Vec3f& free_local_goal, const double& max_dis = 999999,
                        const vec_Vec3i& neighbor_list = vec_Vec3i{}) const;

        bool isLineFree(const Vec3f& start_pt, const Vec3f& end_pt,
                        const bool& use_inf_map = false,
                        const bool& use_unk_as_occ = false) const;

        bool getNearestCellIs(const GridType& target_type,
                                const Vec3f& start_pos,
                                Vec3f& nearest_pt, const double& max_dis) const {
            return findNearestCellThat(IS, target_type, start_pos, nearest_pt, max_dis);
        }

        bool getNearestCellNot(const GridType& target_type,
                             const Vec3f& start_pos,
                             Vec3f& nearest_pt, const double& max_dis) const {
            return findNearestCellThat(NOT, target_type, start_pos, nearest_pt, max_dis);
        }

        bool getNearestInfCellIs(const GridType& target_type,
                           const Vec3f& start_pos,
                           Vec3f& nearest_pt, const double& max_dis) const {
            return findNearestInfCellThat(IS, target_type, start_pos, nearest_pt, max_dis);
        }

        bool getNearestInfCellNot(const GridType& target_type,
                             const Vec3f& start_pos,
                             Vec3f& nearest_pt, const double& max_dis) const {
            return findNearestInfCellThat(NOT, target_type, start_pos, nearest_pt, max_dis);
        }

        void probMapPosToGlobalIndex(const Vec3f & pos, Vec3i & id_g) const {
            this->posToGlobalIndex(pos, id_g);
        }

        void probMapGlobalIndexToPos(const Vec3i & id_g, Vec3f & pos) const {
            this->globalIndexToPos(id_g, pos);
        }

        void infMapPosToGlobalIndex(const Vec3f & pos, Vec3i & id_g) const {
            inf_map_->infMapPosToGlobalIndex(pos, id_g);
        }

        void infMapGlobalIndexToPos(const Vec3i & id_g, Vec3f & pos) const {
            inf_map_->infMapGlobalIndexToPos(id_g, pos);
        }


        void updateMap(const PointCloud& cloud, const Pose& pose);

        RobotState getRobotState() const;

    protected:

        std::ofstream time_log_file_, map_info_log_file_;

        void updateRobotState(const Pose& pose);

        bool findNearestCellThat(const bool & is, const GridType& target_type,
            const Vec3f & start_pos, Vec3f& nearest_pt, const double & max_dis) const ;

        bool findNearestInfCellThat(const bool & is, const GridType& target_type,
          const Vec3f & start_pos, Vec3f& nearest_pt, const double & max_dis) const ;

        RobotState robot_state_;
    };
}
