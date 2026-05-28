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

#ifndef SUPER_UTILS_HEADER_TYPE_UTILS_HPP
#define SUPER_UTILS_HEADER_TYPE_UTILS_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <algorithm>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <vector>
#include <Eigen/Eigen>
#include <super_utils/type_utils.hpp>
#include <super_utils/fmt_eigen.hpp>


namespace rog_map {
    using super_utils::Vec3f;
    using super_utils::Vec3i;
    using super_utils::vec_Vec3f;
    using super_utils::vec_E;

    template <typename Scalar_t>
    static Eigen::Matrix<Scalar_t, 3, 1> quaternion_to_ypr(const Eigen::Quaternion<Scalar_t>& q_) {
        Eigen::Quaternion<Scalar_t> q = q_.normalized();

        Eigen::Matrix<Scalar_t, 3, 1> ypr;
        ypr(2) = atan2(2 * (q.w() * q.x() + q.y() * q.z()), 1 - 2 * (q.x() * q.x() + q.y() * q.y()));
        ypr(1) = asin(2 * (q.w() * q.y() - q.z() * q.x()));
        ypr(0) = atan2(2 * (q.w() * q.z() + q.x() * q.y()), 1 - 2 * (q.y() * q.y() + q.z() * q.z()));

        return ypr;
    }

    template <typename Scalar_t>
    static Scalar_t get_yaw_from_quaternion(const Eigen::Quaternion<Scalar_t>& q) {
        return quaternion_to_ypr(q)(0);
    }


    static double computePathLength(const super_utils::vec_E<super_utils::Vec3f>& path) {
        if (path.size() < 2) {
            return 0.0;
        }
        double len = 0.0;
        for (size_t i = 0; i < path.size() - 1; i++) {
            len += (path[i] - path[i + 1]).norm();
        }
        return len;
    }

    static super_utils::Vec3f lineBoxIntersectPoint(const super_utils::Vec3f& pt, const super_utils::Vec3f& pos,
                                                    const super_utils::Vec3f& box_min,
                                                    const super_utils::Vec3f& box_max) {
        Eigen::Vector3d diff = pt - pos;
        Eigen::Vector3d max_tc = box_max - pos;
        Eigen::Vector3d min_tc = box_min - pos;

        double min_t = 1000000;

        for (int i = 0; i < 3; ++i) {
            if (fabs(diff[i]) > 0) {
                double t1 = max_tc[i] / diff[i];
                if (t1 > 0 && t1 < min_t)
                    min_t = t1;

                double t2 = min_tc[i] / diff[i];
                if (t2 > 0 && t2 < min_t)
                    min_t = t2;
            }
        }

        return pos + (min_t - 1e-3) * diff;
    }

    static bool GetIntersection(float fDst1, float fDst2, super_utils::Vec3f P1, super_utils::Vec3f P2,
                                super_utils::Vec3f& Hit) {
        if ((fDst1 * fDst2) >= 0.0f) return false;
        if (fDst1 == fDst2) return false;
        Hit = P1 + (P2 - P1) * (-fDst1 / (fDst2 - fDst1));
        return true;
    }

    static bool InBox(super_utils::Vec3f Hit, super_utils::Vec3f B1, super_utils::Vec3f B2, const int Axis) {
        if (Axis == 1 && Hit.z() > B1.z() && Hit.z() < B2.z() && Hit.y() > B1.y() && Hit.y() < B2.y()) return true;
        if (Axis == 2 && Hit.z() > B1.z() && Hit.z() < B2.z() && Hit.x() > B1.x() && Hit.x() < B2.x()) return true;
        if (Axis == 3 && Hit.x() > B1.x() && Hit.x() < B2.x() && Hit.y() > B1.y() && Hit.y() < B2.y()) return true;
        return false;
    }

    //The box in this article is Axis-Aligned and so can be defined by only two 3D points:
    // B1 - the smallest values of X, Y, Z
    //        B2 - the largest values of X, Y, Z
    // returns true if line (L1, L2) intersects with the box (B1, B2)
    // returns intersection point in Hit
    static bool lineIntersectBox(super_utils::Vec3f L1, super_utils::Vec3f L2, super_utils::Vec3f B1,
                                 super_utils::Vec3f B2, super_utils::Vec3f& Hit) {
        if (L2.x() < B1.x() && L1.x() < B1.x()) return false;
        if (L2.x() > B2.x() && L1.x() > B2.x()) return false;
        if (L2.y() < B1.y() && L1.y() < B1.y()) return false;
        if (L2.y() > B2.y() && L1.y() > B2.y()) return false;
        if (L2.z() < B1.z() && L1.z() < B1.z()) return false;
        if (L2.z() > B2.z() && L1.z() > B2.z()) return false;

        if ((GetIntersection(L1.x() - B1.x(), L2.x() - B1.x(), L1, L2, Hit) && InBox(Hit, B1, B2, 1))
            || (GetIntersection(L1.y() - B1.y(), L2.y() - B1.y(), L1, L2, Hit) && InBox(Hit, B1, B2, 2))
            || (GetIntersection(L1.z() - B1.z(), L2.z() - B1.z(), L1, L2, Hit) && InBox(Hit, B1, B2, 3))
            || (GetIntersection(L1.x() - B2.x(), L2.x() - B2.x(), L1, L2, Hit) && InBox(Hit, B1, B2, 1))
            || (GetIntersection(L1.y() - B2.y(), L2.y() - B2.y(), L1, L2, Hit) && InBox(Hit, B1, B2, 2))
            || (GetIntersection(L1.z() - B2.z(), L2.z() - B2.z(), L1, L2, Hit) && InBox(Hit, B1, B2, 3)))
            return true;

        return false;
    }
}


#endif //SUPER_UTILS_HEADER_TYPE_UTILS_HPP