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

#include <data_structure/base/trajectory.h>
#include <utils/optimization/lbfgs.h>


namespace optimization_utils {

    using namespace geometry_utils;
    using super_utils::PolyhedronH;
    using super_utils::PolyhedraH;
    using super_utils::PolyhedraV;
    using super_utils::PolyhedronV;

    template<typename EIGENVEC>
    class Gcopter {
    public:
        Gcopter() = default;

        ~Gcopter() = default;

        static bool smoothedL1(const double &x,
                               const double &mu,
                               double &f,
                               double &df);


        static void forwardMapTauToT(const Eigen::VectorXd &tau,
                                     Eigen::VectorXd &T);

        static void backwardMapTToTau(const Eigen::VectorXd &T, EIGENVEC &tau);

        static void propagateGradientTToTau(const Eigen::VectorXd &tau,
                                            const Eigen::VectorXd &gradT,
                                            EIGENVEC &gradTau);

        static void mapIntervalToInf(const double &lower_bound,
                                     const double &upper_bound,
                                     const double &inter,
                                     double &inf);

        static void mapInfToInterval(const double &lower_bound,
                                     const double &upper_bound,
                                     const double &inf,
                                     double &inter);

        static void propagateGradIntervalToInf(const double &lower_bound,
                                               const double &upper_bound,
                                               const double &inf,
                                               const double &grad_inter,
                                               double &grad_inf);


        // For corridor optimization
        static void normRetrictionLayer(const Eigen::VectorXd &xi,
                                        const PolyhedronV &vPoly,
                                        double &cost,
                                        EIGENVEC &gradXi);

        static void backwardGradP(const Eigen::VectorXd &xi, const PolyhedronV &vPoly, const Eigen::Matrix3Xd &gradP,
                                  EIGENVEC &gradXi);

        static void backwardGradP(const Eigen::VectorXd &xi,
                                         const Eigen::VectorXi &vIdx,
                                         const PolyhedraV &vPolys,
                                         const Eigen::Matrix3Xd &gradP,
                                         EIGENVEC &gradXi);

        static void backwardP(const Eigen::Matrix3Xd &points, const PolyhedronV &vPoly, EIGENVEC &xi);

        static void forwardP(const Eigen::VectorXd &xi, const PolyhedronV &vPoly, Eigen::Matrix3Xd &P);

        static double costTinyNLS(void *ptr,
                                  const Eigen::VectorXd &xi,
                                  Eigen::VectorXd &gradXi);

        static void forwardP(const Eigen::VectorXd &xi,
                             const Eigen::VectorXi &vIdx,
                             const PolyhedraV &vPolys,
                             Eigen::Matrix3Xd &P);

        static void backwardP(const Eigen::Matrix3Xd &P,
                              const Eigen::VectorXi &vIdx,
                              const PolyhedraV &vPolys,
                              EIGENVEC &xi);

        static void normRetrictionLayer(const Eigen::VectorXd &xi,
                                        const Eigen::VectorXi &vIdx,
                                        const PolyhedraV &vPolys,
                                        double &cost,
                                        EIGENVEC &gradXi);
    };

    typedef Gcopter<Eigen::Map<Eigen::VectorXd>> gcopter;
}

