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

#ifndef GEOMETRY_UTILS_TRAJECTORY_H
#define GEOMETRY_UTILS_TRAJECTORY_H

#include <data_structure/base/piece.h>
#include "utils/header/color_msg_utils.hpp"

namespace geometry_utils {

    class Trajectory {
        typedef std::vector<Piece> Pieces;
        Pieces pieces;

        // serial method for logging
    public:
        template <class Archive>
        void serialize(Archive& archive) {
            archive(start_WT, pieces);
        }


        double start_WT{0}; // start wall time

        Trajectory() = default;

        Trajectory(const std::vector<double>& durs,
                   const std::vector<Eigen::MatrixXd>& cMats);

        Trajectory operator+(const Trajectory& traj_in) const {
            Trajectory new_traj;
            new_traj.pieces.reserve(pieces.size() + traj_in.pieces.size());
            new_traj.pieces.insert(new_traj.pieces.end(), pieces.begin(), pieces.end());
            new_traj.pieces.insert(new_traj.pieces.end(),
                                   std::make_move_iterator(traj_in.pieces.begin()),
                                   std::make_move_iterator(traj_in.pieces.end()));
            new_traj.start_WT = start_WT;
            return new_traj;
        }


        bool empty() const;

        int getPieceNum() const;

        vec_Vec3f getWaypoints() const;

        Eigen::VectorXd getDurations() const;

        double getTotalDuration() const;

        const Piece& operator[](int i) const {
            return pieces[i];
        }

        Piece& operator[](int i) {
            return pieces[i];
        }

        void clear();

        Pieces::const_iterator begin() const {
            return pieces.begin();
        }

        Pieces::const_iterator end() const {
            return pieces.end();
        }

        Pieces::iterator begin() {
            return pieces.begin();
        }

        Pieces::iterator end() {
            return pieces.end();
        }

        size_t size() const {
            return pieces.size();
        }

        void reserve(const int& n);

        void emplace_back(const Piece& piece);

        void emplace_back(const double& dur,
                          const Eigen::MatrixXd& cMat);

        void append(const Trajectory& traj);

        int locatePieceIdx(double& t) const;

        double getWaypointTT(const int& watpoint_id) const;

        Eigen::Vector3d getPos(double t) const;

        Eigen::Vector3d getVel(double t) const;

        Eigen::Vector3d getAcc(double t) const;

        Eigen::Vector3d getJer(double t) const;

        Mat3Df getState(double t) const;

        bool getState(double t, StatePVAJ & state) const;

        Vec3f getSnap(double t) const;

        Eigen::Vector3d getJuncPos(int juncIdx) const;

        Eigen::Vector3d getJuncVel(int juncIdx) const;

        Eigen::Vector3d getJuncAcc(int juncIdx) const;

        bool getPartialTrajectoryByID(const int& start_id, const int& end_id, Trajectory& out_traj) const;

        bool getPartialTrajectoryByTime(const double& start_t, const double& end_t, Trajectory& out_traj) const;

        double getMaxVelRate() const;

        double getMaxAccRate() const;

        bool checkMaxVelRate(const double& maxVelRate) const;

        bool checkMaxAccRate(const double& maxAccRate) const;

        void printProfile() const;
    };
}
#endif
