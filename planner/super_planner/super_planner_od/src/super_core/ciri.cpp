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

#include <super_core/ciri.h>
using namespace color_text;
using namespace optimization_utils;
using namespace super_utils;


namespace super_planner {

    RET_CODE CIRI::comvexDecomposition(const Eigen::MatrixX4d& bd, const Eigen::Matrix3Xd& pc, const Eigen::Vector3d& a,
                                       const Eigen::Vector3d& b) {
        const Eigen::Vector4d ah(a(0), a(1), a(2), 1.0);
        const Eigen::Vector4d bh(b(0), b(1), b(2), 1.0);

        /// force return if the seed is not inside the boundary
        if ((bd * ah).maxCoeff() > epsilon_ ||
            (bd * bh).maxCoeff() > epsilon_) {
//            cout << YELLOW << " -- [WARN] ah, bh not in BD, forced return." << endl;
//            cout << "bd  * ah: " << (bd * ah).transpose().maxCoeff() << endl;
//            cout << "bd  * bh: " << (bd * bh).transpose().maxCoeff() << endl;
            return INIT_ERROR;
        }


        /// Maximum M boundary constraints and N point constraints
        const int M = bd.rows();
        const int N = pc.cols();

        Ellipsoid E(Mat3f::Identity(), (a + b) / 2);
        if ((a - b).norm() > 0.1) {
            /// use line seed
            findEllipsoid(pc, a, b, E);
        }

        vector<Eigen::Vector4d> planes;
        MatD4f hPoly;

//        bool infeasible_problem{false};
        Vec3f infeasible_pt_w;

        for (int loop = 0; loop < iter_num_; ++loop) {
            // Initialize the boundary in ellipsoid frame
            const Eigen::Vector3d fwd_a = E.toEllipsoidFrame(a);
            const Eigen::Vector3d fwd_b = E.toEllipsoidFrame(b);
            const Eigen::MatrixX4d bd_e = E.toEllipsoidFrame(bd);
            const Eigen::VectorXd distDs = bd_e.rightCols<1>().cwiseAbs().cwiseQuotient(
                    bd_e.leftCols<3>().rowwise().norm());
            const Eigen::Matrix3Xd pc_e = E.toEllipsoidFrame(pc);
            Eigen::VectorXd distRs = pc_e.colwise().norm();

            Eigen::Matrix<uint8_t, -1, 1> bdFlags = Eigen::Matrix<uint8_t, -1, 1>::Constant(M, 1);
            Eigen::Matrix<uint8_t, -1, 1> pcFlags = Eigen::Matrix<uint8_t, -1, 1>::Constant(N, 1);

            bool completed = false;
            int bdMinId, pcMinId;
            double minSqrD = distDs.minCoeff(&bdMinId);
            double minSqrR = distRs.minCoeff(&pcMinId);

            Eigen::Vector4d temp_tangent, temp_plane_w;
            const Mat3f C_inv = E.C().inverse();

            planes.clear();
            planes.reserve(30);
            Vec3f tmp_nn_pt;
            Vec4f plan_before_ab;

            for (int i = 0; !completed && i < (M + N); ++i) {
                if (minSqrD < minSqrR) {
                    /// Case [Bd closer than ob]  enable the boundary constrain.
                    Vec4f p_e = bd_e.row(bdMinId);
                    temp_plane_w = E.toWorldFrame(p_e);
                    bdFlags(bdMinId) = 0;
                }
                else {
                    /// Case [Ob closer than Bd] enable the obstacle point constarin.
                    ///     Compute the tangent plane of sphere
                    ///
                    const auto & pt_w = pc.col(pcMinId);
                    const auto dis = distancePointToSegment(pt_w,a,b);
                    if(dis < robot_r_ - 1e-2) {
//                        infeasible_problem = true;
                        infeasible_pt_w = pt_w;
                        cout<<YELLOW<<" -- [CIRI] WARNING! The problem is not feasible, the min dis to obstacle is only: "<<dis<<RESET<<endl;
                        return FAILED;
                        cout<<" -- [CIRI] dis: "<<dis<<endl;
                        cout<<" -- [CIRI] robot_r: "<<robot_r_<<endl;
                        cout<<" -- [CIRI] pcMin: "<<pt_w.transpose()<<endl;
                        cout<<" -- [CIRI] a: "<<a.transpose()<<endl;
                        cout<<" -- [CIRI] b: "<<b.transpose()<<endl;
                    }

                    if (robot_r_ < epsilon_) {
                        const Vec3f& pt_e = pc_e.col(pcMinId);
                        temp_tangent(3) = -distRs(pcMinId);
                        temp_tangent.head(3) = pt_e.transpose() / distRs(pcMinId);

                        if (temp_tangent.head(3).dot(fwd_a) + temp_tangent(3) > epsilon_) {
                            const Eigen::Vector3d delta = pc_e.col(pcMinId) - fwd_a;
                            temp_tangent.head(3) = fwd_a - (delta.dot(fwd_a) / delta.squaredNorm()) * delta;
                            distRs(pcMinId) = temp_tangent.head(3).norm();
                            temp_tangent(3) = -distRs(pcMinId);
                            temp_tangent.head(3) /= distRs(pcMinId);
                        }
                        if (temp_tangent.head(3).dot(fwd_b) + temp_tangent(3) > epsilon_) {
                            const Eigen::Vector3d delta = pc_e.col(pcMinId) - fwd_b;
                            temp_tangent.head(3) = fwd_b - (delta.dot(fwd_b) / delta.squaredNorm()) * delta;
                            distRs(pcMinId) = temp_tangent.head(3).norm();
                            temp_tangent(3) = -distRs(pcMinId);
                            temp_tangent.head(3) /= distRs(pcMinId);
                        }
                        if (temp_tangent.head(3).dot(fwd_b) + temp_tangent(3) > epsilon_) {
                            const Eigen::Vector3d delta = pc_e.col(pcMinId) - fwd_b;
                            temp_tangent.head(3) = fwd_b - (delta.dot(fwd_b) / delta.squaredNorm()) * delta;
                            distRs(pcMinId) = temp_tangent.head(3).norm();
                            temp_tangent(3) = -distRs(pcMinId);
                            temp_tangent.head(3) /= distRs(pcMinId);
                        }
                        temp_plane_w = E.toWorldFrame(temp_tangent);
                    }
                    else {
                        /// Case [Ob closer than Bd] enable the obstacle point constarin.
                        const Vec3f &pt_e = pc_e.col(pcMinId);
                        const Vec3f &pt_w = pc.col(pcMinId);
                        Ellipsoid E_pe(C_inv * sphere_template_.C(), pt_e);
                        Vec3f close_pt_e;
                        E_pe.pointDistaceToEllipsoid(Vec3f(0, 0, 0), close_pt_e);
                        Vec3f c_pt_w = E.toWorldFrame(close_pt_e);
                        temp_plane_w.head(3) = (pt_w - c_pt_w).normalized();
                        temp_plane_w(3) = -temp_plane_w.head(3).dot(c_pt_w);

                        /// Cut line with sphere A and B,
                        if (temp_plane_w.head(3).dot(a) + temp_plane_w(3) > -epsilon_) {
                            // Case the plan make seed out, the plane should be modified in world frame
                            findTangentPlaneOfSphere(pt_w, robot_r_, a, E.d(), temp_plane_w);
                        } else if (temp_plane_w.head(3).dot(b) + temp_plane_w(3) > -epsilon_) {
                            // Case the plan make seed out, the plane should be modified in world frame
                            findTangentPlaneOfSphere(pt_w, robot_r_, b, E.d(), temp_plane_w);
                        }
                    }
                    pcFlags(pcMinId) = 0;
                    tmp_nn_pt = pc.col(pcMinId);
                }
                // update pcMinId and bdMinId
                completed = true;
                minSqrD = INFINITY;
                for (int j = 0; j < M; ++j) {
                    if (bdFlags(j)) {
                        completed = false;
                        if (minSqrD > distDs(j)) {
                            bdMinId = j;
                            minSqrD = distDs(j);
                        }
                    }
                }
                minSqrR = INFINITY;
                for (int j = 0; j < N; ++j) {
                    if (pcFlags(j)) {
                        if ((temp_plane_w.head(3).dot(pc.col(j)) + temp_plane_w(3)) > robot_r_ - epsilon_) {
                            pcFlags(j) = 0;
                        }
                        else {
                            completed = false;
                            if (minSqrR > distRs(j)) {
                                pcMinId = j;
                                minSqrR = distRs(j);
                            }
                        }
                    }
                }
                planes.push_back(temp_plane_w);
            }

            hPoly.resize(planes.size(), 4);
            for (size_t i = 0; i < planes.size(); ++i) {
                hPoly.row(i) = planes[i];
            }

            if (loop == iter_num_ - 1) {
                break;
            }

            if(hPoly.array().isNaN().any()) {
                cout << YELLOW << " -- [CIRI] ERROR! maxVolInsEllipsoid failed." << RESET << endl;
//                optimized_polytope_.Reset();
//                optimized_polytope_.SetPlanes(hPoly);
//                optimized_polytope_.SetSeedLine(std::make_pair(a, b));
//                optimized_polytope_.SetEllipsoid(E);
//                ros_ptr_->vizCiriSeedLine(a, b,robot_r_);
//                cout<<" -- [CIRI] infeasible_pt_w: "<<infeasible_pt_w.transpose()<<endl;
//                ros_ptr_->vizCiriInfeasiblePoint(infeasible_pt_w);
//                ros_ptr_->vizCiriEllipsoid(E);
////                optimized_polytope_.Visualize(debug_pub_, "optimized_polytope");
//                std::cout << " -- [CIRI] hPoly: " << hPoly << std::endl;
                return FAILED;
            }

            if (!MVIE::maxVolInsEllipsoid(hPoly, E)) {
                return FAILED;
            }
        }

        if (std::isnan(hPoly.sum())) {
            cout << YELLOW << " -- [CIRI] ERROR! There is nan in generated planes." << RESET << endl;
            cout << a.transpose() << endl;
            cout << b.transpose() << endl;
            ros_ptr_->vizCiriSeedLine(a, b,robot_r_);
            ros_ptr_->vizCiriEllipsoid(E);
            return FAILED;
        }
        Vec3f inner;
        if (!geometry_utils::findInterior(hPoly, inner)) {
            cout<<RED<<" -- [CIRI] The polytope is empty."<<RESET<<endl;
            ros_ptr_->vizCiriSeedLine(a, b,robot_r_);
            ros_ptr_->vizCiriEllipsoid(E);
            return FAILED;
        }
        optimized_polytope_.Reset();
        optimized_polytope_.SetPlanes(hPoly);
        optimized_polytope_.SetSeedLine(std::make_pair(a, b));
        optimized_polytope_.SetEllipsoid(E);

        return SUCCESS;
    }

    void CIRI::getPolytope(Polytope& optimized_poly) {
        optimized_poly = optimized_polytope_;
    }

    void CIRI::setupParams(double robot_r, int iter_num) {
        robot_r_ = robot_r;
        iter_num_ = iter_num;
        sphere_template_ = Ellipsoid(Mat3f::Identity(), robot_r_ * Vec3f(1, 1, 1), Vec3f(0, 0, 0));
        //        split_seed_max_ = split_seed_max;
        //        split_thresh_ = split_thresh;
    }

    void CIRI::findTangentPlaneOfSphere(const Eigen::Vector3d& center, const double& r,
                                        const Eigen::Vector3d& pass_point,
                                        const Eigen::Vector3d& seed_p,
                                        Eigen::Vector4d& outter_plane) {

        // v2
        // const Vec3f v1 = (pass_point - seed_p);
        // const Vec3f v2 = (center - seed_p);
        // const Vec3f v3 = (center - pass_point);
        // const double d1 = v1.norm();
        // const double d2 = v2.norm();
        // const double d3 = v3.norm();
        //
        // if(d3 < r || d1 < 1e-2 || d2< 1e-2) {
        //     cout<<YELLOW<<" -- [CIRI] findTangentPlaneOfSphere: The pass point is inside the sphere."<<RESET<<endl;
        //     return;
        // }
        //
        // const double theta = asin(r / d3);
        // const Vec3f axis = v1.cross(v2);
        // const double d = sqrt(d3 * d3 - r * r);
        // Eigen::AngleAxis<decimal_t> rot(theta, axis.normalized());
        // Vec3f new_tangent_p = (rot * v3).normalized() * d + pass_point;
        // Vec3f new_tangent_n = (new_tangent_p - center).normalized();
        // // rebuild the tangent plan form pass point and tangent_normal
        // outter_plane.head(3) = new_tangent_n;
        // outter_plane(3) = -new_tangent_n.dot(new_tangent_p);
        // if (outter_plane.head(3).dot(seed_p) + outter_plane(3) > epsilon_) {
        //     outter_plane = -outter_plane;
        // }

        Vec3f seed = seed_p;
        Vec3f dif = pass_point - pass_point;
        if (dif.norm() < 1e-3) {
            if ((pass_point - center).head(2).norm() > 1e-3) {
                Vec3f v1 = (pass_point - center).normalized();
                v1(2) = 0;
                seed = seed_p + 0.01 * v1.cross(Vec3f(0, 0, 1)).normalized();
            }
            else {
                seed = seed_p + 0.01 * (pass_point - center).cross(Vec3f(1, 0, 0)).normalized();
            }
        }
        Eigen::Vector3d P = pass_point - center;
        Eigen::Vector3d norm_ = (pass_point - center).cross(seed - center).normalized();
        Eigen::Matrix3d R = Eigen::Quaterniond::FromTwoVectors(norm_, Vec3f(0, 0, 1)).matrix();
        P = R * P;
        Eigen::Vector3d C = R * (seed - center);
        Eigen::Vector3d Q;
        double r2 = r * r;
        double p1p2n = P.head(2).squaredNorm();
        double d = sqrt(p1p2n - r2);
        double rp1p2n = r / p1p2n;
        double q11 = rp1p2n * (P(0) * r - P(1) * d);
        double q21 = rp1p2n * (P(1) * r + P(0) * d);

        double q12 = rp1p2n * (P(0) * r + P(1) * d);
        double q22 = rp1p2n * (P(1) * r - P(0) * d);
        if (q11 * C(0) + q21 * C(1) < 0) {
            Q(0) = q12;
            Q(1) = q22;
        }
        else {
            Q(0) = q11;
            Q(1) = q21;
        }
        Q(2) = 0;
        // point(Q) + normal (AQ)
        outter_plane.head(3) = R.transpose() * Q;
        Q = outter_plane.head(3) + center;
        outter_plane(3) = -Q.dot(outter_plane.head(3));
        if (outter_plane.head(3).dot(seed) + outter_plane(3) > epsilon_) {
            outter_plane = -outter_plane;
        }
    }

    void CIRI::findEllipsoid(const Eigen::Matrix3Xd& pc,
                             const Eigen::Vector3d& a,
                             const Eigen::Vector3d& b,
                             Ellipsoid& out_ell) {
        double f = (a - b).norm() / 2;
        Mat3f C = f * Mat3f::Identity();
        Vec3f r = Vec3f::Constant(f);
        Vec3f center = (a + b) / 2;
        C(0, 0) += robot_r_;
        r(0) += robot_r_;
        if (r(0) > 0) {
            double ratio = r(1) / r(0);
            r *= ratio;
            C *= ratio;
        }

        Mat3f Ri = Eigen::Quaterniond::FromTwoVectors(Vec3f::UnitX(), (b - a)).toRotationMatrix();
        Ellipsoid E(Ri, r, center);
        Mat3f Rf = Ri;
        Mat3Df obs;
        int min_dis_id;
        Vec3f pw;
        if (E.pointsInside(pc, obs, min_dis_id)) {
            pw = obs.col(min_dis_id);
        }
        else {
            out_ell = E;
            return;
        }
        Mat3Df obs_inside = obs;
        int max_iter = 100;
        while (max_iter--) {
            Vec3f p_e = Ri.transpose() * (pw - E.d());
            const double roll = atan2(p_e(2), p_e(1));
            Rf = Ri * Eigen::Quaterniond(cos(roll / 2), sin(roll / 2), 0, 0);
            p_e = Rf.transpose() * (pw - E.d());
            if (p_e(0) < r(0)) {
                r(1) = std::abs(p_e(1)) / std::sqrt(1 - std::pow(p_e(0) / r(0), 2));
            }
            E = Ellipsoid(Rf, r, center);
            if (E.pointsInside(obs_inside, obs_inside, min_dis_id)) {
                pw = obs_inside.col(min_dis_id);
            }
            else {
                break;
            }
        }
        if (max_iter == 0) {
            cout << YELLOW << " -- [CIRI] Find Ellipsoid reach max iteration, may cause error." << endl;
        }
        max_iter = 100;


        if (E.pointsInside(obs, obs_inside, min_dis_id)) {
            pw = obs_inside.col(min_dis_id);
        }
        else {
            out_ell = E;
            return;
        }

        while (max_iter--) {
            Vec3f p = Rf.transpose() * (pw - E.d());
            double dd = 1 - std::pow(p(0) / r(0), 2) -
                        std::pow(p(1) / r(1), 2);
            if (dd > epsilon_) {
                r(2) = std::abs(p(2)) / std::sqrt(dd);
            }
            E = Ellipsoid(Rf, r, center);
            if (E.pointsInside(obs_inside, obs_inside, min_dis_id)) {
                pw = obs_inside.col(min_dis_id);
            }
            else {
                out_ell = E;
                break;
            }
        }

        if (max_iter == 0) {
            cout << YELLOW << " -- [CIRI] Find Ellipsoid reach max iteration, may cause error." << endl;
        }
        E = Ellipsoid(Rf, r, center);
        out_ell = E;
    }
}
