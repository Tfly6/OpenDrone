/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <Eigen/Core>

const size_t nStates = 9;
const size_t nControls = 4;

typedef Eigen::Matrix<double, nStates, 1> state_vector_t;
typedef Eigen::Matrix<double, nControls, 1> control_vector_t;

typedef Eigen::Matrix<double, nStates, nStates> state_matrix_t;
typedef Eigen::Matrix<double, nControls, nControls> control_matrix_t;
typedef Eigen::Matrix<double, nStates, nControls> control_gain_matrix_t;

//static const std::string LQR_Dir = "/home/llanesc/catkin_ws/src/lqr_controller";
