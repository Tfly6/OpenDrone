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

#include <super_core/corridor_generator.h>

using namespace super_utils;

namespace super_planner {

    CorridorGenerator::CorridorGenerator(const ros_interface::RosInterface::Ptr &ros_ptr,
                                         const rog_map::ROGMapROS::Ptr &map_ptr, const double bound_dis,
                                         const double seed_line_max_dis, const double min_overlap_threshold,
                                         const double virtual_groud_height, const double virtual_ceil_height,
                                         const double robot_r, const int box_search_skip_num, const int iris_iter_num)
            : ros_ptr_(ros_ptr), map_ptr_(map_ptr) {
        ciri_ = std::make_shared<CIRI>(ros_ptr_);
        ciri_->setupParams(robot_r, iris_iter_num);
        bound_dis_ = bound_dis;
        seed_line_max_length_ = seed_line_max_dis;
        min_overlap_threshold_ = min_overlap_threshold;
        robot_r_ = robot_r;
        box_search_skip_num_ = box_search_skip_num;
        iris_iter_num_ = iris_iter_num;
        virtual_ceil_height_ = virtual_ceil_height - robot_r;
        virtual_groud_height_ = virtual_groud_height + robot_r;
//        failed_traj_log.open(DEBUG_FILE_DIR("sfc.csv"), std::ios::out | std::ios::trunc);
    }


    void CorridorGenerator::SetLineNeighborList(const vec_E<Vec3i> &_line_seed_neighbor_list) {
        this->line_seed_neighbor_list = _line_seed_neighbor_list;
    }

    bool
    CorridorGenerator::SearchPolytopeOnPath(const vec_Vec3f &path, PolytopeVec &sfcs,
                                            Vec3f &shifted_start_pt,
                                            bool cut_first_poly) {
        // https://whimsical.com/flow-3TASJFwe1dASYYY2xHEmze
        // password: wtr
        //	TimeConsuming t___("SearchPolytopeOnPath");
        sfcs.clear();
        if (path.empty()) {
            return false;
        }

        vector<Line> seed_lines;
        int first_id, second_id;
        Polytope overlap;
        Vec3f interior_pt;
        double interior_depth;
        Polytope temp_poly, temp_poly_fix_p;
        int max_loop = 1000;
        int cnt_loop = 0;
        first_id = 0;

        while(first_id < path.size() && map_ptr_->isOccupiedInflate(path[first_id])) {
            first_id++;
        }

        if(first_id!=0){
            shifted_start_pt = path[first_id];
            double dis = (path[first_id] - path[0]).norm() * 1.2;
            GenerateEmptyPolytope(path[0], dis, temp_poly);
            sfcs.emplace_back(temp_poly);
        }

        while (cnt_loop++ < max_loop) {
            second_id = first_id;
            for (int j = first_id + 1; j < path.size(); j++) {
                bool reach_segment = false;
                if (!map_ptr_->isLineFree(path[first_id], path[j], seed_line_max_length_,
                                          line_seed_neighbor_list)) {
                    reach_segment = true;
                }
                if (reach_segment) {
                    second_id = j - 1;
                    if (second_id - 1 > first_id) {
                        second_id -= 1;
                    }
                    break;
                }
                second_id = j;
            }

            if (second_id == first_id && second_id + 1 < path.size()) {
                second_id += 1;
            }

            seed_lines.emplace_back(path[first_id], path[second_id]);
            if ((path[first_id] - path[second_id]).norm() > seed_line_max_length_ * 1.5) {
                fmt::print("first: {}\n second: {}\n seed line max: {}\n", path[first_id].transpose(),
                           path[second_id].transpose(), seed_line_max_length_);
                throw std::runtime_error("seed line too long");
                return false;
            }
            if (!GeneratePolytopeFromLine(seed_lines.back(), temp_poly)) {
                cout << YELLOW << " -- [SUPER] GeneratePolytopeFromLine failed." << RESET << endl;
                return false;
            }

// viz for debug
//            ros_ptr_->vizCiriPolytope(temp_poly, "debug");
//            usleep(10000);

            if (!sfcs.empty()) {
                overlap = sfcs.back().CrossWith(temp_poly);
                interior_depth = geometry_utils::findInteriorDist(overlap.GetPlanes(), interior_pt);
                temp_poly.overlap_depth_with_last_one = interior_depth;
                temp_poly.interior_pt_with_last_one = interior_pt;
                if (interior_depth < min_overlap_threshold_) {
                    if (!GeneratePolytopeFromPoint(path[first_id], temp_poly_fix_p)) {
                        cout << YELLOW << " -- [SUPER] GeneratePolytopeFromPoint failed." << RESET << endl;
                        return false;
                    }
                    overlap = sfcs.back().CrossWith(temp_poly_fix_p);
                    interior_depth = geometry_utils::findInteriorDist(overlap.GetPlanes(), interior_pt);
                    if (interior_depth <= 0.01) {
                        ros_ptr_->warn(
                                " -- [SUPER] Cannot find continuous corridor on path, overlap only {}, force return.",
                                interior_depth);
// viz for debug
//                        ros_ptr_->vizCiriPointCloud(latest_pc);
//                        usleep(100000);
//                        exit(-1);
                        return false;
                    }
                    temp_poly_fix_p.overlap_depth_with_last_one = interior_depth;
                    temp_poly_fix_p.interior_pt_with_last_one = interior_pt;
                    sfcs.push_back(temp_poly_fix_p);
                    overlap = sfcs.back().CrossWith(temp_poly);
                    interior_depth = geometry_utils::findInteriorDist(overlap.GetPlanes(), interior_pt);
                    if (interior_depth <= 0.01) {
                        ros_ptr_->warn(
                                " -- [SUPER] Cannot find continuous corridor on path, overlap only {}, force return.",
                                interior_depth);
                        // viz for debug
//                        ros_ptr_->vizCiriPointCloud(latest_pc);
//                        usleep(100000);
//                        exit(-1);
                        return false;
                    }
                } else {
                    int temp_id = sfcs.size() - 2;
                    if (temp_id > 0) {
                        overlap = sfcs[temp_id].CrossWith(temp_poly);
                        interior_depth = geometry_utils::findInteriorDist(overlap.GetPlanes(), interior_pt);
                        if (interior_depth > sfcs[temp_id + 1].overlap_depth_with_last_one * 0.25) {
                            temp_poly.overlap_depth_with_last_one = interior_depth;
                            temp_poly.interior_pt_with_last_one = interior_pt;
                            sfcs.pop_back();
                        }
                    }
                }
            }

            sfcs.push_back(temp_poly);
            if (second_id == path.size() - 1) {
                break;
            }
            first_id = second_id;
        }
        // Delete last polytope if the second last one contains the last point


        if (cnt_loop >= max_loop) {
            cout << YELLOW << " -- [SUPER] Reach max iteration, failed." << RESET << endl;
            return false;
        }

        if (sfcs.empty()) {
            return false;
        }

        return true;
    }


    void CorridorGenerator::getSeedBBox(const Vec3f &p1, const Vec3f &p2, Vec3f &box_min, Vec3f &box_max) {
        box_min = p1.cwiseMin(p2);
        box_max = p1.cwiseMax(p2);
        box_min -= Vec3f(bound_dis_, bound_dis_, bound_dis_);
        box_max += Vec3f(bound_dis_, bound_dis_, bound_dis_);
//        box_min.z() = std::max(box_min.z(), virtual_groud_height_);
//        box_max.z() = std::min(box_max.z(), virtual_ceil_height_);
    }

    bool CorridorGenerator::GeneratePolytopeFromPoint(const Vec3f &pt, Polytope &polytope) {
        Eigen::Vector3d box_max, box_min;
        vec_E<Vec3f> pc;
        getSeedBBox(pt, pt, box_min, box_max);
        // TODO the box did not consider the robot_r
        map_ptr_->boundBoxByLocalMap(box_min, box_max);
        map_ptr_->boxSearch(box_min, box_max, OCCUPIED, pc);
        box_min.z() += robot_r_;
        box_max.z() -= robot_r_;
        MatD4f planes;
        Eigen::Vector3d a = pt, b = pt;
        Eigen::Matrix<double, 6, 4> bd = Eigen::Matrix<double, 6, 4>::Zero();
        bd(0, 0) = 1.0;
        bd(1, 0) = -1.0;
        bd(2, 1) = 1.0;
        bd(3, 1) = -1.0;
        bd(4, 2) = 1.0;
        bd(5, 2) = -1.0;
        bd(0, 3) = -box_max.x();
        bd(1, 3) = box_min.x();
        bd(2, 3) = -box_max.y();
        bd(3, 3) = box_min.y();
        bd(4, 3) = -box_max.z();
        bd(5, 3) = box_min.z();
        // 将vector放到Eigen里，准备开始分解
        if (pc.empty()) {
            // 障碍物点云为空，直接返回一个方块
            // Ax + By + Cz + D = 0
            planes.resize(6, 4);
            planes.row(0) << 1, 0, 0, -box_max.x();
            planes.row(1) << 0, 1, 0, -box_max.y();
            planes.row(2) << 0, 0, 1, -box_max.z();
            planes.row(3) << -1, 0, 0, box_min.x();
            planes.row(4) << 0, -1, 0, box_min.y();
            planes.row(5) << 0, 0, -1, box_min.z();
            polytope.SetPlanes(planes);
            polytope.SetSeedLine(Line{pt, pt});
            return true;
        }
        latest_pc.insert(latest_pc.end(), pc.begin(), pc.end());
        Eigen::Map<const Eigen::Matrix<double, 3, -1, Eigen::ColMajor>> pp(pc[0].data(), 3, pc.size());
        rog_map::TimeConsuming tc("emvp", false);
        RET_CODE success = ciri_->comvexDecomposition(bd, pp, a, b);
        double dt = tc.stop();
        if (success == SUCCESS) {
            ciri_cnt++;
            ciri_t += dt;
            ciri_->getPolytope(polytope);
            polytope.SetSeedLine(Line{pt, pt});
            return true;
        } else {
            cout << YELLOW << " -- [SUPER] CSpaceFiri failed." << RESET << endl;
            cout << YELLOW << "\t box_min =" << box_min.transpose() << endl;
            cout << YELLOW << "\t box_max = " << box_max.transpose() << endl;
            cout << YELLOW << "\t seed pt = " << pt.transpose() << endl;
            polytope.Reset();
            return false;
        }

    }

    bool CorridorGenerator::GenerateEmptyPolytope(const super_utils::Vec3f &pt,
                                                  const double & dis,
                                                  Polytope & polytope){
        Eigen::Vector3d box_max, box_min;
        box_min = pt;
        box_max = pt;
        box_min -= Vec3f(dis, dis, dis);
        box_max += Vec3f(dis, dis, dis);
        MatD4f planes;
        Eigen::Matrix<double, 6, 4> bd = Eigen::Matrix<double, 6, 4>::Zero();
        bd(0, 0) = 1.0;
        bd(1, 0) = -1.0;
        bd(2, 1) = 1.0;
        bd(3, 1) = -1.0;
        bd(4, 2) = 1.0;
        bd(5, 2) = -1.0;
        bd(0, 3) = -box_max.x();
        bd(1, 3) = box_min.x();
        bd(2, 3) = -box_max.y();
        bd(3, 3) = box_min.y();
        bd(4, 3) = -box_max.z();
        bd(5, 3) = box_min.z();
        // 障碍物点云为空，直接返回一个方块
        // Ax + By + Cz + D = 0
        planes.resize(6, 4);
        planes.row(0) << 1, 0, 0, -box_max.x();
        planes.row(1) << 0, 1, 0, -box_max.y();
        planes.row(2) << 0, 0, 1, -box_max.z();
        planes.row(3) << -1, 0, 0, box_min.x();
        planes.row(4) << 0, -1, 0, box_min.y();
        planes.row(5) << 0, 0, -1, box_min.z();
        polytope.SetPlanes(planes);
        polytope.SetSeedLine(Line{pt, pt});
        return true;
    }

    bool CorridorGenerator::GeneratePolytopeFromLine(Line &line, Polytope &polytope) {
        Eigen::Vector3d box_max, box_min;
        vec_E<Vec3f> pc, pts{line.first, line.second};
        getSeedBBox(line.first, line.second, box_min, box_max);
        map_ptr_->boundBoxByLocalMap(box_min, box_max);
        map_ptr_->boxSearch(box_min, box_max, OCCUPIED, pc);
        box_min.z() += robot_r_;
        box_max.z() -= robot_r_;
        MatD4f planes;
        Eigen::Vector3d a = line.first, b = line.second;
        Eigen::Matrix<double, 6, 4> bd = Eigen::Matrix<double, 6, 4>::Zero();
        bd(0, 0) = 1.0;
        bd(1, 0) = -1.0;
        bd(2, 1) = 1.0;
        bd(3, 1) = -1.0;
        bd(4, 2) = 1.0;
        bd(5, 2) = -1.0;
        bd(0, 3) = -box_max.x();
        bd(1, 3) = box_min.x();
        bd(2, 3) = -box_max.y();
        bd(3, 3) = box_min.y();
        bd(4, 3) = -box_max.z();
        bd(5, 3) = box_min.z();
        // 将vector放到Eigen里，准备开始分解
        if (pc.empty()) {
            // 障碍物点云为空，直接返回一个方块
            // Ax + By + Cz + D = 0g
            planes.resize(6, 4);
            planes.row(0) << 1, 0, 0, -box_max.x();
            planes.row(1) << 0, 1, 0, -box_max.y();
            planes.row(2) << 0, 0, 1, -box_max.z();
            planes.row(3) << -1, 0, 0, box_min.x();
            planes.row(4) << 0, -1, 0, box_min.y();
            planes.row(5) << 0, 0, -1, box_min.z();
            polytope.SetPlanes(planes);
            polytope.SetSeedLine(line);
            return true;
        }
        // save to latest pc
        latest_pc.insert(latest_pc.end(), pc.begin(), pc.end());
        Eigen::Map<const Eigen::Matrix<double, 3, -1, Eigen::ColMajor>> pp(pc[0].data(), 3, pc.size());
        rog_map::TimeConsuming tc("emvp", false);
        RET_CODE success = ciri_->comvexDecomposition(bd, pp, a, b);
        double dt = tc.stop();
        if (success == SUCCESS) {
            ciri_cnt++;
            ciri_t += dt;
            ciri_->getPolytope(polytope);
            polytope.SetSeedLine(line);
            return true;
        } else {
            polytope.Reset();
            cout << YELLOW << "\t box_min = " << box_min.transpose() << RESET << endl;
            cout << YELLOW << "\t box_max =" << box_max.transpose() << RESET << endl;
            cout << YELLOW << "\t seed line =" << line.first.transpose() << " --> " << line.second.transpose()
                 << RESET << endl;

//            failed_traj_log << 889900 << endl;
//            failed_traj_log << bd << endl;
//            failed_traj_log << 0 << endl;
//            failed_traj_log << pp << endl;
//            failed_traj_log << 0 << endl;
//            failed_traj_log << a.transpose() << endl;
//            failed_traj_log << 0 << endl;
//            failed_traj_log << b.transpose() << endl;
//            failed_traj_log << 0 << endl;
//            failed_traj_log << robot_r_ << endl;
//            failed_traj_log << 0 << endl;
//            failed_traj_log << iris_iter_num_ << endl;
            return false;
        }

    }
}