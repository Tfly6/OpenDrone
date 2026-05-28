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

#ifndef SUPER_EXP_TRAJ_OPT_H
#define SUPER_EXP_TRAJ_OPT_H

#include <iostream>
#include <vector>

#include <traj_opt/config.hpp>
#include <traj_opt/minco.h>


#include <data_structure/base/polytope.h>
#include <data_structure/base/trajectory.h>

#include <utils/header/scope_timer.hpp>
#include <utils/header/type_utils.hpp>
#include <utils/optimization/optimization_utils.h>
#include <utils/geometry/geometry_utils.h>

#include <ros_interface/ros_interface.hpp>

namespace traj_opt {

    using super_utils::MatD3f;
    using super_utils::Mat3Df;
    using super_utils::VecDi;
    using super_utils::VecDf;
    using super_utils::PolyhedraH;
    using super_utils::PolyhedraV;


    class ExpTrajOpt {
        traj_opt::Config cfg_;
        std::ofstream failed_traj_log;
        std::ofstream penalty_log;
        ros_interface::RosInterface::Ptr ros_ptr_;

        struct OptimizationVariables {
            double rho;
            int iter_num{0};
            int pos_constraint_type;
            bool block_energy_cost;
            double smooth_eps;
            int integral_res;
            flatness::FlatnessMap quadrotor_flatness;

            Mat3Df gradByPoints;
            VecDf gradByTimes;
            MatD3f partialGradByCoeffs;
            VecDf partialGradByTimes;
            bool default_init{true};
            bool given_init_ts_and_ps{false};
            int piece_num;
            Mat3Df points;
            VecDf times;
            VecDf magnitudeBounds, penaltyWeights;

            PolyhedraV vPolytopes; // the original sfc and intersecting sfc
            PolyhedraH hPolytopes; // the original sfc
            PolyhedraH hOverlapPolytopes;
            Mat3Df init_path;
            VecDf init_ts;
            vec_Vec3f init_ps;
            Mat3Df waypoint_attractor;
            VecDf waypoint_attractor_dead_d;

            VecDi pieceIdx;
            VecDi vPolyIdx;
            VecDi hPolyIdx;

            MINCO_S4NU minco;

            StatePVAJ headPVAJ;
            StatePVAJ tailPVAJ;
            vec_E<Vec3f> guide_path;
            vector<double> guide_t;

            int temporalDim, spatialDim;

            VecDf penalty_log;
        } opt_vars;

        static double costFunctional(void *ptr,
                                     const VecDf &x,
                                     VecDf &g);

        static void constraintsFunctional(const VecDf &T,
                                          const MatD3f &coeffs,
                                          const VecDi &hIdx,
                                          const PolyhedraH &hPolys,
                                          const Mat3Df &waypoint_attractor,
                                          const VecDf &waypoint_attractor_dead_d,
                                          const double &smoothFactor,
                                          const int &integralResolution,
                                          const VecDf &magnitudeBounds,
                                          const VecDf &penaltyWeights,
                                          flatness::FlatnessMap &flatMap,
                                          double &cost,
                                          VecDf &gradT,
                                          MatD3f &gradC,
                                          VecDf &penalty_log);

        bool processCorridor();

        bool processCorridorWithGuideTraj();

        void defaultInitialization();

        bool setupProblemAndCheck();

        bool processCorridorWithGuideTraj2() {
            using namespace traj_opt;
            using namespace color_text;
            using namespace super_utils;
            using namespace math_utils;
            using namespace optimization_utils;
            // * 1) allocate memory for vertex
            const int sizeCorridor = static_cast<int>(opt_vars.hPolytopes.size() - 1);

            opt_vars.vPolytopes.clear();
            opt_vars.vPolytopes.reserve(2 * sizeCorridor + 1);

            long nv;
            PolyhedronH curIH;
            PolyhedronV curIV, curIOB;
            opt_vars.waypoint_attractor.resize(3, sizeCorridor);
            opt_vars.hOverlapPolytopes.resize(sizeCorridor);
            opt_vars.waypoint_attractor_dead_d.resize(sizeCorridor);
            // * 2) Process the corridor
            for (int i = 0; i < sizeCorridor; i++) {
                // * 2.1) Get current vertex
                if (!geometry_utils::enumerateVs(opt_vars.hPolytopes[i], curIV)) {
                    cout << YELLOW << " -- [SUPER] in [ GcopterExpS4::processCorridor]: Failed to enumerate corridor Vs."
                         << RESET << endl;

                    return false;
                }
                // * 2.3) Conver the vertex to the frame of the first point
                nv = curIV.cols();
                curIOB.resize(3, nv);
                // *    Save the position of the first point
                curIOB.col(0) = curIV.col(0);
                // *    Use the relative position of the rest vertex.
                curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
                // *    save the i-th corridor's vertex
                opt_vars.vPolytopes.push_back(curIOB);

                // * 2.4) Find the overlap corridor
                curIH.resize(opt_vars.hPolytopes[i].rows() + opt_vars.hPolytopes[i + 1].rows(), 4);
                curIH.topRows(opt_vars.hPolytopes[i].rows()) = opt_vars.hPolytopes[i];
                curIH.bottomRows(opt_vars.hPolytopes[i + 1].rows()) = opt_vars.hPolytopes[i + 1];
                opt_vars.hOverlapPolytopes[i] = curIH;
                Vec3f interior;

                const double dis = geometry_utils::findInteriorDist(curIH, interior) / 2;
                if (dis < 0.0 || std::isinf(dis)) {

                    cout << YELLOW << " -- [SUPER] in [ GcopterExpS4::processCorridor]: Failed findInteriorDist Vs." <<
                         RESET << endl;
                    return false;
                }
                geometry_utils::enumerateVs(curIH, interior, curIV);
                const double test_sum = curIV.sum();
                if (std::isnan(test_sum) || std::isinf(test_sum)) {
                    return false;
                }
                opt_vars.waypoint_attractor.col(i) = curIV.rowwise().mean();
                opt_vars.waypoint_attractor_dead_d(i) = dis;
                nv = curIV.cols();
                curIOB.resize(3, nv);
                curIOB.col(0) = curIV.col(0);
                curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
                opt_vars.vPolytopes.push_back(curIOB);
            }

            // * 3) Time and waypoint allocation for hot initialization
            VecDf min_dis(opt_vars.waypoint_attractor.cols());
            VecDi min_id(opt_vars.waypoint_attractor.cols());
            VecDf time_stamps(opt_vars.waypoint_attractor.cols() + 2);
            time_stamps(0) = 0.0;
            time_stamps(opt_vars.waypoint_attractor.cols() + 1) = opt_vars.guide_t.back();
            min_id.setConstant(0);
            min_dis.setConstant(std::numeric_limits<double>::max());
            for (int i = 0; i < opt_vars.guide_path.size(); i++) {
                for (int j = 0; j < opt_vars.waypoint_attractor.cols(); j++) {
                    const double dis = (opt_vars.guide_path[i] - opt_vars.waypoint_attractor.col(j)).norm();
                    if (dis < min_dis[j]) {
                        min_dis[j] = dis;
                        min_id[j] = i;
                        opt_vars.points.col(j) = opt_vars.guide_path[i];
                        time_stamps(j + 1) = opt_vars.guide_t[i];
                    }
                }
            }

            for (int i = 1; i < time_stamps.size(); i++) {
                opt_vars.times(i - 1) = time_stamps(i) - time_stamps(i - 1);
                opt_vars.times(i - 1) = std::max(0.01, opt_vars.times(i - 1));
            }

            if (!geometry_utils::enumerateVs(opt_vars.hPolytopes.back(), curIV)) {
                return false;
            }
            nv = curIV.cols();
            curIOB.resize(3, nv);
            curIOB.col(0) = curIV.col(0);
            curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
            opt_vars.vPolytopes.push_back(curIOB);
            return true;
        }

        bool setupProblemAndCheck2() {
            // init internal variables size;
            opt_vars.piece_num = static_cast<int>(opt_vars.hPolytopes.size());
            opt_vars.times.resize(opt_vars.piece_num);
            opt_vars.points.resize(3, opt_vars.piece_num - 1);


            // Check corridor and init points
            if (opt_vars.default_init) {
                if (!processCorridor()) {
                    return false;
                }
            } else {
                if (!processCorridorWithGuideTraj2()) {
                    return false;
                }
            }
            opt_vars.init_path.resize(3, opt_vars.piece_num + 1);
            for (long i = 0; i < opt_vars.piece_num - 1; i++) {
                opt_vars.init_path.col(i + 1) = opt_vars.waypoint_attractor.col(i);
            }
            opt_vars.init_path.col(0) = opt_vars.headPVAJ.col(0);
            opt_vars.init_path.rightCols(1) = opt_vars.tailPVAJ.col(0);
            if (opt_vars.default_init) {
                defaultInitialization();
            } else {
                opt_vars.times *= 0.8;
            }

            if (std::isnan(opt_vars.times.sum())) {
//                cout << YELLOW << " -- [ExpOpt] Init times and point failed." << RESET << endl;
                return false;
            }

            const Mat3Df deltas = opt_vars.init_path.rightCols(opt_vars.piece_num)
                                  - opt_vars.init_path.leftCols(opt_vars.piece_num);
            opt_vars.pieceIdx = (deltas.colwise().norm() / INFINITY).cast<int>().transpose();
            opt_vars.pieceIdx.array() += 1;

            opt_vars.temporalDim = opt_vars.piece_num;
            opt_vars.spatialDim = 0;
            opt_vars.vPolyIdx.resize(opt_vars.piece_num - 1);
            opt_vars.hPolyIdx.resize(opt_vars.piece_num);

            switch (cfg_.pos_constraint_type) {
                case 1: {
                    for (int i = 0, j = 0, k; i < opt_vars.piece_num; i++) {
                        k = opt_vars.pieceIdx(i);
                        for (int l = 0; l < k; l++, j++) {
                            if (l < k - 1) {
                                opt_vars.vPolyIdx(j) = 2 * i;
                            } else if (i < opt_vars.piece_num - 1) {
                                opt_vars.vPolyIdx(j) = 2 * i + 1;
                            }
                            opt_vars.hPolyIdx(j) = i;
                        }
                    }
                    opt_vars.spatialDim = 3 * (opt_vars.piece_num - 1);
                    break;
                }
                default: {
                    for (int i = 0, j = 0, k; i < opt_vars.piece_num; i++) {
                        k = opt_vars.pieceIdx(i);
                        for (int l = 0; l < k; l++, j++) {
                            if (l < k - 1) {
                                opt_vars.vPolyIdx(j) = 2 * i;
                                opt_vars.spatialDim += static_cast<int>(opt_vars.vPolytopes[2 * i].cols());
                            } else if (i < opt_vars.piece_num - 1) {
                                opt_vars.vPolyIdx(j) = 2 * i + 1;
                                opt_vars.spatialDim += static_cast<int>(opt_vars.vPolytopes[2 * i + 1].cols());
                            }
                            opt_vars.hPolyIdx(j) = i;
                        }
                    }
                }
            }

            opt_vars.minco.setConditions(opt_vars.headPVAJ, opt_vars.tailPVAJ, opt_vars.piece_num);
            opt_vars.gradByPoints.resize(3, opt_vars.piece_num - 1);
            opt_vars.gradByTimes.resize(opt_vars.piece_num);
            opt_vars.partialGradByCoeffs.resize(8 * opt_vars.piece_num, 3);
            opt_vars.partialGradByTimes.resize(opt_vars.piece_num);
            return true;
        }

        bool setInitPsAndTs(const vec_Vec3f &init_ps, const vector<double> &init_ts);

        double optimize(Trajectory &traj, const double &relCostTol);

    public:
        typedef std::shared_ptr<ExpTrajOpt> Ptr;

        ExpTrajOpt(const traj_opt::Config &cfg, const ros_interface::RosInterface::Ptr & ros_ptr);

        ~ExpTrajOpt();

        bool optimize(const StatePVAJ &headPVAJ, const StatePVAJ &tailPVAJ,
                      PolytopeVec &sfcs,
                      Trajectory &out_traj);

        bool optimize(const StatePVAJ &headPVAJ, const StatePVAJ &tailPVAJ,
                      const vec_E<Vec3f> &guide_path, const vector<double> &guide_t,
                      PolytopeVec &sfcs,
                      Trajectory &out_traj);

        void getInitValue(VecDf &ts, vec_Vec3f &ps) const {
            ts = opt_vars.init_ts;
            ps = opt_vars.init_ps;
        }

        bool optimize(const StatePVAJ &headPVAJ, const StatePVAJ &tailPVAJ,
                      PolytopeVec &sfcs,
                      const vec_Vec3f & init_ps,
                      const VecDf & init_ts,
                      Trajectory &out_traj);

    };
}

#endif