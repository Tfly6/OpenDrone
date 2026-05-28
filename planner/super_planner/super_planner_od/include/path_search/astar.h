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

#include "Eigen/Dense"
#include "vector"
#include "rog_map_ros/rog_map_ros1.hpp"
// #include "rog_map_ros/rog_map_ros2.hpp"
#include "queue"
#include "path_search/config.hpp"
#include "utils/header/type_utils.hpp"
#include <ros_interface/ros_interface.hpp>


namespace path_search {
    using namespace super_utils;

    constexpr double inf = 1 >> 20;
    struct GridNode;
    typedef GridNode *GridNodePtr;

    struct GridNode {
        enum enum_state {
            OPENSET = 1,
            CLOSEDSET = 2,
            UNDEFINED = 3
        } state{UNDEFINED};

        int rounds{0};
        rog_map::Vec3i id_g;
        double total_score{inf}, distance_score{inf};
        double distance_to_goal{inf};
        GridNodePtr father_ptr{nullptr};
    };

    class NodeComparator {
    public:
        bool operator()(GridNodePtr node1, GridNodePtr node2) {
            return node1->total_score > node2->total_score;
        }
    };

    class FrontierComparator {
    public:
        bool operator()(GridNodePtr node1, GridNodePtr node2) {
            return node1->distance_to_goal > node2->distance_to_goal;
        }
    };

    const int ON_INF_MAP = (1 << 0);
    const int ON_PROB_MAP = (1 << 1);
    const int UNKNOWN_AS_OCCUPIED = (1 << 3);
    const int UNKNOWN_AS_FREE = (1 << 4);
    const int USE_INF_NEIGHBOR = (1 << 5);
    const int DONT_USE_INF_NEIGHBOR = (1 << 6);

    class Astar {

        rog_map::ROGMapROS::Ptr map_ptr_;
        ros_interface::RosInterface::Ptr ros_ptr_;

        PathSearchConfig cfg_;
        const double tie_breaker_ = 1.0 + 1e-5;
        rog_map::vec_Vec3i sorted_pts;
        rog_map::vec_Vec3i neighbor_list;

        vector<GridNodePtr> grid_node_buffer_;

        int rounds_{0};

        static constexpr int DIAG = 0;
        static constexpr int MANH = 1;
        static constexpr int EUCL = 2;

        struct MissionData {
            rog_map::Vec3f start_pt;
            rog_map::Vec3f goal_pt;
            double searching_horizon;
            bool use_inf_map{false};
            bool use_prob_map{false};
            bool unknown_as_occ{false};
            bool unknown_as_free{false};
            bool use_inf_neighbor{false};
            double resolution;
            rog_map::Vec3i local_map_center_id_g;
            rog_map::Vec3f local_map_center_d;
            double mission_rcv_WT{0};
            rog_map::Vec3f local_map_max_d, local_map_min_d;
            std::mutex mission_mtx;
        } md_;



        double getHeu(GridNodePtr node1, GridNodePtr node2, int type = DIAG) const;

         int getLocalIndexHash(const rog_map::Vec3i &id_in) const;

        void posToGlobalIndex(const rog_map::Vec3f &pos, rog_map::Vec3i &id_g) const ;

        void globalIndexToPos(const rog_map::Vec3i &id_g, rog_map::Vec3f &pos) const;

        bool insideLocalMap(const rog_map::Vec3f &pos) const;

        bool insideLocalMap(const rog_map::Vec3i &id_g) const;

        bool neighborHaveOne(const rog_map::GridType &type, const rog_map::Vec3i &src_id);

        RET_CODE setup(const rog_map::Vec3f &start_pt, const rog_map::Vec3f &goal_pt, const int &flag,
                       const double &searching_horizon = 9999);

        void retrievePath(GridNodePtr current, vector<GridNodePtr> &path);

        void ConvertNodePathToPointPath(const vector<GridNodePtr> &node_path, rog_map::vec_Vec3f &point_path);

    public:

        Astar(const std::string & cfg_path,
              const ros_interface::RosInterface::Ptr &ros_ptr,
              rog_map::ROGMapROS::Ptr rm);

        ~Astar() {};

        typedef std::shared_ptr<Astar> Ptr;

        void setVisualProcessEn(const bool &en);

        void setFineInfNeighbors(const int & neighbor_step);

        RET_CODE pointToPointPathSearch(const rog_map::Vec3f &start_pt, const rog_map::Vec3f &end_pt,
                                        const int &flag,
                                        const double &searching_horizon,
                                        rog_map::vec_Vec3f &out_path,
                                        const double &time_out = 0.1);

        /// @ brief: The escape path only for path search from prob map to inf map. from non-occupied point to
        ///          inf map free (or known freee) point . Aim to find a path from current point to (known) free point
        /// @ param:
        RET_CODE escapePathSearch(const rog_map::Vec3f &start_pt, const int flag, rog_map::vec_Vec3f &out_path);


    };
}