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

/*
    MIT License

    Copyright (c) 2021 Zhepei Wang (wangzhepei@live.com)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

#pragma once

#include <utils/header/type_utils.hpp>
#include <utils/geometry/quickhull.h>
#include <Eigen/Eigen>


namespace geometry_utils {
    using super_utils::Mat3f;
    using super_utils::Vec3f;
    using super_utils::Vec4f;
    using super_utils::vec_Vec3f;
    using super_utils::Mat3Df;
    using super_utils::MatD4f;
    using super_utils::vec_E;
    using super_utils::PolyhedronH;

    ///============ 2023-06-30: add by yunfan ============///
    static void simplePMTimeAllocator(const double &a_max, const double &v_max,
                                const double &v0,
                                const double &total_dis,
                                const double &cur_dis, double &t, double &vel) {
        // Helper lambda functions
        auto calc_dis = [](double a, double t) { return 0.5 * a * t * t; };
        auto calc_time = [](double a, double cur_dis) { return sqrt(2 * cur_dis / a); };
        auto solve_quadratic = [](double a, double b, double c) {
            double delta = b * b - 4 * a * c;
            return (-b + sqrt(delta)) / (2 * a);
        };

        // Precompute reusable values
        const double t_to_v_max = v_max / a_max;
        const double dis_to_v_max = calc_dis(a_max, t_to_v_max);

        const double t_to_v0 = v0 / a_max;
        const double dis_to_v0 = calc_dis(a_max, t_to_v0);

        const double dec_time = (v_max - v0) / a_max;
        const double dec_dis = 0.5 * (v_max + v0) * dec_time;

        // Case 1: Only acceleration to v0
        if (total_dis <= dis_to_v0) {
            t = calc_time(a_max, cur_dis);
            vel = a_max * t;
            return;
        }

        // Case 2: Acceleration to v_max, then deceleration
        if (total_dis <= dis_to_v_max + dec_dis) {
            const double a = 2 * a_max;
            const double b = -(a_max - v0);
            const double c = -(v0 * v0 / a_max + 2 * total_dis);
            const double t_acc = solve_quadratic(a, b, c);
            const double t_dec = t_acc - t_to_v0;
            const double dis_acc = calc_dis(a_max, t_acc);
            const double cur_v_max = a_max * t_acc;

            if (cur_dis <= dis_acc) {
                t = calc_time(a_max, cur_dis);
                vel = a_max * t;
            } else {
                const double remaining_dis = cur_dis - dis_acc;
                const double t2 = solve_quadratic(-a_max, 2 * cur_v_max, -2 * remaining_dis);
                t = t_acc + t2;
                vel = cur_v_max - t2 * a_max;
            }
            return;
        }

        // Case 3: Acceleration + constant speed + deceleration
        if (cur_dis < dis_to_v_max) {  // Case 3.1: During acceleration phase
            t = calc_time(a_max, cur_dis);
            vel = a_max * t;
            return;
        }

        if (cur_dis < total_dis - dec_dis) {  // Case 3.2: During constant speed phase
            const double remaining_dis = cur_dis - dis_to_v_max;
            const double t_const = remaining_dis / v_max;
            t = t_to_v_max + t_const;
            vel = v_max;
            return;
        }

        // Case 3.3: During deceleration phase
        const double const_phase_dis = total_dis - dec_dis - dis_to_v_max;
        const double remaining_dis = cur_dis - dis_to_v_max - const_phase_dis;
        const double t_dec = solve_quadratic(-a_max, 2 * v_max, -2 * remaining_dis);
        t = t_to_v_max + const_phase_dis / v_max + t_dec;
        vel = v_max - t_dec * a_max;
    }

    ///============ 2023-06-30: add by yunfan ============///
    double DistancePointEllipse(double e0, double e1, double y0, double y1, double& x0, double& x1);

    double
    DistancePointEllipsoid(double e0, double e1, double e2, double y0, double y1, double y2, double& x0, double& x1,
                           double& x2);


    ///============ 2023-06-13: add by Gene ============///
    template <typename Scalar_t>
    Eigen::Matrix<Scalar_t, 3, 1> quaternion_to_yrp(const Eigen::Quaternion<Scalar_t>& q_);

    ///============ 2023-06-13: add by Yunfan ============///
    Vec4f translatePlane(const Vec4f& plane, const Vec3f& translation);

    ///============ 2023-05-23: add by Yunfan ============///
    void normalizeNextYaw(const double& last_yaw, double& yaw);

    ///============ 2023-3-12: add by Yunfan ============///
    void convertFlatOutputToAttAndOmg(const Vec3f& p,
                                      const Vec3f& v,
                                      const Vec3f& a,
                                      const Vec3f& j,
                                      const double& yaw,
                                      const double& yaw_dot,
                                      Vec3f& rpy,
                                      Vec3f& omg,
                                      double& aT
    );


    ///============ 2022-12-10: add by Yunfan ============///
    bool pointInsidePolytope(const Vec3f& point, const PolyhedronH& polytope,
                             double margin = 1e-6);

    ///============ 2022-12-5: add by Yunfan ============///
    double pointLineSegmentDistance(const Vec3f& p, const Vec3f& a, const Vec3f& b);

    double computePathLength(const vec_E<Vec3f>& path);

    ///============ 2022-11-18 ====================================================================================
    int inline GetIntersection(float fDst1, float fDst2, Vec3f P1, Vec3f P2, Vec3f& Hit);

    int inline InBox(Vec3f Hit, Vec3f B1, Vec3f B2, const int Axis);

    //The box in this article is Axis-Aligned and so can be defined by only two 3D points:
    // B1 - the smallest values of X, Y, Z
    //        B2 - the largest values of X, Y, Z
    // returns true if line (L1, L2) intersects with the box (B1, B2)
    // returns intersection point in Hit
    int lineIntersectBox(Vec3f L1, Vec3f L2, Vec3f B1, Vec3f B2, Vec3f& Hit);

    Vec3f lineBoxIntersectPoint(const Vec3f& pt, const Vec3f& pos,
                                const Vec3f& box_min, const Vec3f& box_max);

    ///================================================================================================


    Eigen::Matrix3d RotationFromVec3(const Eigen::Vector3d& v);

    // 通过三点获得一个平面
    void FromPointsToPlane(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& p3,
                           Eigen::Vector4d& hPoly);

    void getFovCheckPlane(const Eigen::Matrix3d R, const Eigen::Vector3d t, Eigen::MatrixX4d& fov_planes,
                          std::vector<Eigen::Matrix3d>& fov_pts);

    void GetFovPlanes(const Eigen::Matrix3d R, const Eigen::Vector3d t, Eigen::MatrixX4d& fov_planes,
                      std::vector<Eigen::Matrix3d>& fov_pts);


    double findInteriorDist(const Eigen::MatrixX4d& hPoly,
                            Eigen::Vector3d& interior);

    // Each row of hPoly is defined by h0, h1, h2, h3 as
    // h0*x + h1*y + h2*z + h3 <= 0
    bool findInterior(const Eigen::MatrixX4d& hPoly,
                      Eigen::Vector3d& interior);

    bool overlap(const Eigen::MatrixX4d& hPoly0,
                 const Eigen::MatrixX4d& hPoly1,
                 const double eps = 1.0e-6);

    struct filterLess {
        bool operator()(const Eigen::Vector3d& l,
                        const Eigen::Vector3d& r) const {
            return l(0) < r(0) ||
            (l(0) == r(0) &&
                (l(1) < r(1) ||
                    (l(1) == r(1) &&
                        l(2) < r(2))));
        }
    };

    void filterVs(const Eigen::Matrix3Xd& rV,
                  const double& epsilon,
                  Eigen::Matrix3Xd& fV);

    // Each row of hPoly is defined by h0, h1, h2, h3 as
    // h0*x + h1*y + h2*z + h3 <= 0
    // proposed epsilon is 1.0e-6
    void enumerateVs(const Eigen::MatrixX4d& hPoly,
                     const Eigen::Vector3d& inner,
                     Eigen::Matrix3Xd& vPoly,
                     const double epsilon = 1.0e-6);

    // Each row of hPoly is defined by h0, h1, h2, h3 as
    // h0*x + h1*y + h2*z + h3 <= 0
    // proposed epsilon is 1.0e-6
    bool enumerateVs(const Eigen::MatrixX4d& hPoly,
                     Eigen::Matrix3Xd& vPoly,
                     const double epsilon = 1.0e-6);


    template <typename Scalar_t>
    Scalar_t toRad(const Scalar_t& x);

    template <typename Scalar_t>
    Scalar_t toDeg(const Scalar_t& x);

    template <typename Scalar_t>
    Eigen::Matrix<Scalar_t, 3, 3> rotx(Scalar_t t);

    template <typename Scalar_t>
    Eigen::Matrix<Scalar_t, 3, 3> roty(Scalar_t t);

    template <typename Scalar_t>
    Eigen::Matrix<Scalar_t, 3, 3> rotz(Scalar_t t);

    template <typename Derived>
    Eigen::Matrix<typename Derived::Scalar, 3, 3> ypr_to_R(const Eigen::DenseBase<Derived>& ypr);

    template <typename Derived>
    Eigen::Matrix<typename Derived::Scalar, 3, 3>
    vec_to_R(const Eigen::MatrixBase<Derived>& v1, const Eigen::MatrixBase<Derived>& v2);

    template <typename Derived>
    Eigen::Matrix<typename Derived::Scalar, 3, 1> R_to_ypr(const Eigen::DenseBase<Derived>& R);

    template <typename Derived>
    Eigen::Quaternion<typename Derived::Scalar> ypr_to_quaternion(const Eigen::DenseBase<Derived>& ypr);

    template <typename Scalar_t>
    Eigen::Matrix<Scalar_t, 3, 1> quaternion_to_ypr(const Eigen::Quaternion<Scalar_t>& q_);

    template <typename Scalar_t>
    Scalar_t get_yaw_from_quaternion(const Eigen::Quaternion<Scalar_t>& q);

    template <typename Scalar_t>
    Eigen::Quaternion<Scalar_t> yaw_to_quaternion(Scalar_t yaw);

    template <typename Scalar_t>
    Scalar_t normalize_angle(Scalar_t a);

    template <typename Scalar_t>
    Scalar_t angle_add(Scalar_t a, Scalar_t b);

    template <typename Scalar_t>
    Scalar_t yaw_add(Scalar_t a, Scalar_t b);

    template <typename Derived>
    Eigen::Matrix<typename Derived::Scalar, 3, 3> get_skew_symmetric(const Eigen::DenseBase<Derived>& v);

    template <typename Derived>
    Eigen::Matrix<typename Derived::Scalar, 3, 1> from_skew_symmetric(const Eigen::DenseBase<Derived>& M);
}
