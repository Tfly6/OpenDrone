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

#include <memory>

#include <data_structure/base/polytope.h>
#include <data_structure/base/ellipsoid.h>

#include <utils/optimization/sdlp.h>
#include <utils/geometry/geometry_utils.h>
#include <utils/optimization/optimization_utils.h>
#include <utils/optimization/mvie.h>
#include <utils/header/type_utils.hpp>

#include <ros_interface/ros_interface.hpp>

namespace super_planner {
    using super_utils::RET_CODE;
    using geometry_utils::Ellipsoid;
    using geometry_utils::Polytope;

    class CIRI {
        ros_interface::RosInterface::Ptr ros_ptr_;
        double robot_r_{0};
        int iter_num_{1};
        bool debug_en{false};

        Ellipsoid sphere_template_;
        Polytope optimized_polytope_;

        std::ofstream failed_log;

/**
 * @brief findEllipsoid: find maximum ellipsoid with RILS
 * @param pc the obstacle points
 * @param a the start point of the line segment seed
 * @param b the end point of the line segment seed
 * @param out_ell the output ellipsoid
 * @param r_robot the robot_size, decide if the polytope need to be shrink
 * @param _fix_p decide if the ellipsoid center need to be optimized
 * @param iterations number of the alternating optimization
 */
        void findEllipsoid(
                const Eigen::Matrix3Xd &pc,
                const Eigen::Vector3d &a,
                const Eigen::Vector3d &b,
                Ellipsoid &out_ell);

        static void findTangentPlaneOfSphere(const Eigen::Vector3d &center, const double &r,
                                             const Eigen::Vector3d &pass_point,
                                             const Eigen::Vector3d &seed_p,
                                             Eigen::Vector4d &outter_plane);

        static double distancePointToSegment(const Eigen::Vector3d& P, const Eigen::Vector3d& A, const Eigen::Vector3d& B) {
            Eigen::Vector3d AB = B - A;
            Eigen::Vector3d AP = P - A;

            double AB_AB = AB.dot(AB);
            double AP_AB = AP.dot(AB);
            double t = AP_AB / AB_AB;

            if (t < 0.0f) {
                return (P - A).norm();
            } else if (t > 1.0f) {
                return (P - B).norm();
            } else {
                Eigen::Vector3d Q = A + t * AB;
                return (P - Q).norm();
            }
        }

    public:
        CIRI() = default;

        CIRI(const ros_interface::RosInterface::Ptr & ros_ptr):ros_ptr_(ros_ptr){
            debug_en = true;
        }

        ~CIRI() = default;

        typedef std::shared_ptr<CIRI> Ptr;

        void setupParams(double robot_r, int iter_num);

        RET_CODE comvexDecomposition(const Eigen::MatrixX4d &bd,
                                     const Eigen::Matrix3Xd &pc,
                                     const Eigen::Vector3d &a,
                                     const Eigen::Vector3d &b);

        void getPolytope(Polytope &optimized_poly);
    };
}
