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

#include <utils/header/type_utils.hpp>
#include <utils/geometry/geometry_utils.h>
#include <data_structure/base/ellipsoid.h>

namespace geometry_utils {

    using super_utils::Mat3f;
    using super_utils::Vec3f;
    using super_utils::vec_Vec3f;
    using super_utils::Mat3Df;
    using super_utils::MatD4f;
    using super_utils::vec_E;



    class Polytope {
        bool undefined{true};
        bool is_known_free{false};

        MatD4f planes;
        bool have_seed_line{false};
    public:
        template <class Archive>
        void serialize(Archive& archive) {
            archive(undefined, is_known_free, planes, have_seed_line,
                    overlap_depth_with_last_one, interior_pt_with_last_one,
                    ellipsoid_, seed_line.first,seed_line.second, robot_r);
        }


        double overlap_depth_with_last_one{0};
        Vec3f interior_pt_with_last_one{};
        Ellipsoid ellipsoid_{};
        std::pair<Vec3f, Vec3f> seed_line{};
        double robot_r{};

        Polytope() = default;

        explicit Polytope(MatD4f _planes);

        bool empty() const;

        bool HaveSeedLine() const;

        void SetSeedLine(const std::pair<Vec3f, Vec3f> &_seed_line, double r = 0);

        int SurfNum() const;

        Polytope CrossWith(const Polytope &b) const ;

        Vec3f CrossCenter(const Polytope &b) const ;

        bool HaveOverlapWith(Polytope cmp, double eps = 1e-6);

        MatD4f GetPlanes() const;

        void Reset();

        bool IsKnownFree();

        void SetKnownFree(bool is_free);

        void SetPlanes(MatD4f _planes);

        void SetEllipsoid(const Ellipsoid &ellip);

        bool PointIsInside(const Vec3f &pt, const double & margin = 0.01) const;

        double GetVolume() const ;

        double volume() const {
            return GetVolume();
        }

    };

    typedef std::vector<Polytope> PolytopeVec;


    static bool SimplifySFC(const Vec3f& head_p, const Vec3f& tail_p,
                                 geometry_utils::PolytopeVec& sfcs) {
        vec_Vec3f path{head_p, tail_p};
        int start_id{-1}, end_id{-1};
        if (sfcs.size() > 2) {
            for (int i = 0; i < sfcs.size(); i++) {
                if (sfcs[i].PointIsInside(path.front())) {
                    start_id = i;
                }
                if (sfcs[sfcs.size() - 1 - i].PointIsInside(path.back())) {
                    end_id = static_cast<int>(sfcs.size() - 1 - i);
                }
            }
            if (start_id < 0 || end_id < 0) {
                std::cout << color_text::RED << " -- [EXPTrajOpt] Ill corridor! Forced return." <<  color_text::
                RESET << std::endl;
                return false;
            }
            if (start_id >= end_id) {
                end_id = start_id;
            }
            PolytopeVec sfcs_new(sfcs.begin() + start_id, sfcs.begin() + end_id + 1);
            if (sfcs_new.size() > 2) {
                Polytope check_cand = sfcs_new[0], last_overlapped = sfcs_new[1];
                PolytopeVec sfcs_final;
                sfcs_final.push_back(sfcs_new[0]);
                for (int i = 2; i < sfcs_new.size(); i++) {
                    Polytope cross_poly = check_cand.CrossWith(sfcs_new[i]);
                    Vec3f interior_pt;
                    bool is_overlapped = geometry_utils::findInterior(cross_poly.GetPlanes(), interior_pt);
                    if (is_overlapped) {
                        last_overlapped = sfcs_new[i];
                        if (last_overlapped.PointIsInside(path.back())) {
                            sfcs_final.push_back(last_overlapped);
                            break;
                        }
                    }
                    else {
                        sfcs_final.push_back(last_overlapped);
                        check_cand = last_overlapped;
                        i--;
                    }
                }
                sfcs = sfcs_final;
            }
            else {
                sfcs = sfcs_new;
            }
        }
        return true;
    }
}

