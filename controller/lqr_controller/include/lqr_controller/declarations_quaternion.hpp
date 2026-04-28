/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <Eigen/Core>

const size_t nStatesQuaternion = 10;
const size_t nControlsQuaternion = 4;

typedef Eigen::Matrix<double, nStatesQuaternion, 1> state_vector_quat_t;
typedef Eigen::Matrix<double, nControlsQuaternion, 1> control_vector_quat_t;

typedef Eigen::Matrix<double, nStatesQuaternion, nStatesQuaternion> state_matrix_quat_t;
typedef Eigen::Matrix<double, nControlsQuaternion, nControlsQuaternion> control_matrix_quat_t;
typedef Eigen::Matrix<double, nStatesQuaternion, nControlsQuaternion> control_gain_matrix_quat_t;

//static const std::string LQR_Dir = "/home/llanesc/catkin_ws/src/lqr_controller";
