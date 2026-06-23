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

#include <utils/geometry/geometry_utils.h>

namespace super_planner {
    using namespace geometry_utils;

    enum FOVType {
        UNDEFINED,
        CONE = 1,
        OMNI = 2
    };


    class FOVChecker {
        Mat3Df four_pts_;
        Mat3Df sixteen_pts_;

    public:
        struct FOVConfig {
            FOVType type;
            // for OMNI
            double upper_angle;
            double lower_angle;
            // for CONE
            double angle;
        } cfg_;

        typedef std::shared_ptr<FOVChecker> Ptr;

        bool cutPolyBySensingHorizon( const Vec3f & robot_p, const Vec3f & guide_p,
                                      const double & sensing_horizon, Polytope & poly) const {
            Vec3f dir = (guide_p - robot_p).normalized();
            Vec3f p = robot_p + dir * sensing_horizon;
            Vec4f new_plane;
            new_plane.head(3) = dir;
            new_plane(3) = -dir.dot(p);
            cout<<new_plane.transpose()<<endl;
            auto planes = poly.GetPlanes();
            MatD4f new_planes(planes.rows() + 1, 4);
            new_planes.topRows(planes.rows()) =planes;
            new_planes.bottomRows(1) = new_plane.transpose();
            cout<<"123123124512======================"<<endl;
            cout<<new_planes<<endl;
            poly.SetPlanes(new_planes);
            Vec3f it;
            //// h0*x + h1*y + h2*z + h3 <= 0
            if (!geometry_utils::findInterior(new_planes,it)) {
                return false;
            }
            return true;
        }

        bool cutPolyByFov(const Vec3f & robot_p, const super_utils::Quatf & robot_q,
            const Vec3f & guide_p, Polytope & poly) const {
            const double small_x = 0.1;
            Vec3f guide_p_B = robot_q.matrix().transpose() * (guide_p - robot_p);
            double yaw_in_body_frame = atan2(guide_p_B.y(), guide_p_B.x());
            Mat3f rotation_matrix3;
            rotation_matrix3 = Eigen::AngleAxisd(yaw_in_body_frame, Eigen::Vector3d::UnitZ());
            Mat3f R = robot_q.matrix() * rotation_matrix3;
            MatD4f fov_plane, temp_plane;
            vec_E<Mat3f> fov_plane_pt;
            Vec3f dir = (robot_p - guide_p).normalized();
            getFovCheckPlane(R, robot_p + dir * small_x * 2, fov_plane,
                                           fov_plane_pt);
            temp_plane.resize(poly.SurfNum() + fov_plane.rows(), 4);
            temp_plane << poly.GetPlanes(), fov_plane;
            Vec3f interior;
            if (geometry_utils::findInteriorDist(poly.GetPlanes(), interior) < small_x) {
                return false;
            }
            poly.SetPlanes(temp_plane);
            return true;
        }

        FOVChecker(const FOVType &fov_type,
                   const double &cone_fov_angle,
                   const double &lower_fov_angle,
                   const double &upper_fov_angle) {
            cfg_.type = fov_type;
            cfg_.angle = cone_fov_angle / 180.0 * M_PI;
            cfg_.lower_angle = lower_fov_angle / 180.0 * M_PI;
            cfg_.upper_angle = upper_fov_angle / 180.0 * M_PI;
            switch (cfg_.type) {
                case OMNI: {
                    static const double sqrt2 = sqrt(2);
                    static double zup = sin(upper_fov_angle) * 5;
                    static double rup = cos(upper_fov_angle) * 5;
                    static double zdown = sin(lower_fov_angle) * 5;
                    static double rdown = cos(lower_fov_angle) * 5;
                    four_pts_.resize(3, 4);
                    four_pts_.col(0) = Eigen::Vector3d(rup, -3, zup);
                    four_pts_.col(1) = Eigen::Vector3d(rup, 3, zup);
                    four_pts_.col(2) = Eigen::Vector3d(rdown, -3, zdown);
                    four_pts_.col(3) = Eigen::Vector3d(rdown, 3, zdown);

                    sixteen_pts_.resize(3, 16);
                    sixteen_pts_.col(0) = Eigen::Vector3d(0, rup, zup);
                    sixteen_pts_.col(1) = Eigen::Vector3d(rup / sqrt2, rup / sqrt2, zup);
                    sixteen_pts_.col(2) = Eigen::Vector3d(rup, 0, zup);
                    sixteen_pts_.col(3) = Eigen::Vector3d(rup / sqrt2, -rup / sqrt2, zup);
                    sixteen_pts_.col(4) = Eigen::Vector3d(0, -rup, zup);
                    sixteen_pts_.col(5) = Eigen::Vector3d(-rup / sqrt2, -rup / sqrt2, zup);
                    sixteen_pts_.col(6) = Eigen::Vector3d(-rup, 0, zup);
                    sixteen_pts_.col(7) = Eigen::Vector3d(-rup / sqrt2, rup / sqrt2, zup);

                    sixteen_pts_.col(8) = Eigen::Vector3d(0, rdown, zdown);
                    sixteen_pts_.col(9) = Eigen::Vector3d(rdown / sqrt2, rdown / sqrt2, zdown);
                    sixteen_pts_.col(10) = Eigen::Vector3d(rdown, 0, zdown);
                    sixteen_pts_.col(11) = Eigen::Vector3d(rdown / sqrt2, -rdown / sqrt2, zdown);
                    sixteen_pts_.col(12) = Eigen::Vector3d(0, -rdown, zdown);
                    sixteen_pts_.col(13) = Eigen::Vector3d(-rdown / sqrt2, -rdown / sqrt2, zdown);
                    sixteen_pts_.col(14) = Eigen::Vector3d(-rdown, 0, zdown);
                    sixteen_pts_.col(15) = Eigen::Vector3d(-rdown / sqrt2, rdown / sqrt2, zdown);

                    break;
                }
                default: {
                    throw std::runtime_error("Unsupported FOV type");
                }
            }
        }

        void getAllFovPlane(
                const Mat3f R,
                const Vec3f t,
                vec_E<Mat3f> &fov_pts) const {
            fov_pts.clear();
            Eigen::Matrix3Xd sixteen_pts = (R * sixteen_pts_).colwise() + t;
            Mat3f temp_mat;
            for (int i = 0; i < 7; i++) {
                temp_mat << sixteen_pts.col(i), sixteen_pts.col(i + 1), t;
                fov_pts.push_back(temp_mat);
            }
            temp_mat << sixteen_pts.col(7), sixteen_pts.col(0), t;
            fov_pts.push_back(temp_mat);
            for (int i = 8; i < 15; i++) {
                temp_mat << sixteen_pts.col(i), sixteen_pts.col(i + 1), t;
                fov_pts.push_back(temp_mat);
            }
            temp_mat << sixteen_pts.col(15), sixteen_pts.col(8), t;
            fov_pts.push_back(temp_mat);
        }

        void getFovCheckPlane(
                const Eigen::Matrix3d R,
                const Eigen::Vector3d t,
                Eigen::MatrixX4d &fov_planes,
                vec_E<Eigen::Matrix3d> &fov_pts) const {
            // 只使用上下两个切面来约束。
            fov_planes.resize(2, 4);
            fov_pts.clear();

            static Eigen::Vector3d inner_pt(1, 0, 0);

            Eigen::Matrix3Xd four_pts = (R * four_pts_).colwise() + t;
            Eigen::Vector3d fov_inner_pt = R * inner_pt + t;
            Eigen::Vector4d temp;
            geometry_utils::FromPointsToPlane(four_pts.col(0), four_pts.col(1), t, temp);
            if (temp.head(3).dot(fov_inner_pt) + temp(3) > 0) {
                temp = -temp;
            }

            Eigen::Matrix3d temp_p;
            temp_p << four_pts.col(0), four_pts.col(1), t;
            fov_pts.push_back(temp_p);
            temp_p << four_pts.col(2), four_pts.col(3), t;
            fov_pts.push_back(temp_p);
            fov_planes.row(0) = temp;
            geometry_utils::FromPointsToPlane(four_pts.col(2), four_pts.col(3), t, temp);
            if (temp.head(3).dot(fov_inner_pt) + temp(3) > 0) {
                temp = -temp;
            }
            fov_planes.row(1) = temp;
        }

        void getFovCheckPlane(const Eigen::Matrix3d R, const Eigen::Vector3d t, Eigen::MatrixX4d &fov_planes) {
            vec_E<Mat3f> temp;
            getFovCheckPlane(R, t, fov_planes, temp);
        }


    };

}