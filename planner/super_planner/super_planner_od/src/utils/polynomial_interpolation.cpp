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

#include <utils/optimization/polynomial_interpolation.h>

//using namespace geometry_utils;
//
//template<int dim>
//Trajectory poly_interpo::
//
//template Trajectory poly_interpo::minimumSnapInterpolation<1>(const Eigen::Matrix<double, 1, 4> &init_state,
//                                                           const Eigen::Matrix<double, 1, 4> &goal_state,
//                                                           const Eigen::Matrix<double, 1, -1> &waypoint,
//                                                           const VecDf &ts);
//
//template Trajectory poly_interpo::minimumSnapInterpolation<2>(const Eigen::Matrix<double, 2, 4> &init_state,
//                                                           const Eigen::Matrix<double, 2, 4> &goal_state,
//                                                           const Eigen::Matrix<double, 2, -1> &waypoint,
//                                                           const VecDf &ts);
//
//template Trajectory poly_interpo::minimumSnapInterpolation<3>(const Eigen::Matrix<double, 3, 4> &init_state,
//                                                           const Eigen::Matrix<double, 3, 4> &goal_state,
//                                                           const Eigen::Matrix<double, 3, -1> &waypoint,
//                                                           const VecDf &ts);
