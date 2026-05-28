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


#ifndef EXP_TRAJ_H
#define EXP_TRAJ_H
#include <data_structure/base/trajectory.h>


namespace super_planner {
    using geometry_utils::Trajectory;
    using geometry_utils::PolytopeVec;

    class ExpTraj{
        /* The optimized positional trajectory */
        Trajectory pos_traj_{};

        /* The optimized yaw trajectory */
        Trajectory yaw_traj_{};

        double start_WT_{0.0};

        /* The safe flight corridor */
        PolytopeVec sfc_{};

        /* some flags */
        bool flag_connected_goal_{false};
        bool flag_empty_{true};
        bool flag_whole_known_free_{false};

        /* some part of exp traj may belong to last backup, record this */
        double on_backup_start_TT{-1}, on_backup_end_TT{-1};

    public:
        explicit  ExpTraj() = default;

        void setEmpty() {
            flag_empty_ = true;
        }

        bool empty() const {
            return flag_empty_;
        }

        bool connectedToGoal()const {
            return flag_connected_goal_;
        }

        size_t getSFCSize() const {
            return sfc_.size();
        }

        bool getFirstPartBackupTraj(double & on_backup_traj_start_TT,
            double & on_backup_traj_end_TT) const {
            if(on_backup_start_TT < 0 || on_backup_end_TT < 0 || on_backup_end_TT < on_backup_start_TT) {
                return false;
            }
            on_backup_traj_start_TT = on_backup_start_TT;
            on_backup_traj_end_TT = on_backup_end_TT;
            return true;
        }

        void setTrajectory(const double & start_WT,
            const Trajectory & pos_traj_in,
            const Trajectory & yaw_traj_in,
            const double & _on_backup_start_TT = -1,
            const double & _on_backup_end_TT = -1) {
            pos_traj_ = pos_traj_in;
            yaw_traj_ = yaw_traj_in;
            start_WT_ = start_WT;
            pos_traj_.start_WT = start_WT;
            yaw_traj_.start_WT = start_WT;
            flag_empty_ = false;
            on_backup_start_TT = _on_backup_start_TT;
            on_backup_end_TT = _on_backup_end_TT;
        }

        double getTotalDuration() const {
            return pos_traj_.getTotalDuration();
        }

        Vec3f getPos(const double & t)const {
            return pos_traj_.getPos(t);
        }

        Vec3f getVel(const double & t)const {
            return pos_traj_.getVel(t);
        }

        StatePVAJ getYawState(const double &t)const {
            return yaw_traj_.getState(t);
        }


        void setSFC(const PolytopeVec & sfc) {
            sfc_ = sfc;
        }

        void setGoalConnectedFlag(const bool & _in) {
            flag_connected_goal_ = _in;
        }

        void setWholeTrajKnownFreeFlag(const bool & _in) {
            flag_whole_known_free_ = _in;
        }

        bool wholeTrajKnownFree() const {
            return flag_whole_known_free_;
        }

        const Trajectory & posTraj() const {
            return pos_traj_;
        }

        const Trajectory & yawTraj() const {
            return yaw_traj_;
        }

        const double & getStartWallTime() const {
            return pos_traj_.start_WT;
        }

        bool getPartialTrajectoryByTrajectoryTime(const double & start_t,
            const double & end_t,
            Trajectory & partial_pos_traj,
            Trajectory & partial_yaw_traj)  const {

            if(!pos_traj_.getPartialTrajectoryByTime(start_t,end_t,partial_pos_traj)) {
                return false;
            }

            if(!yaw_traj_.getPartialTrajectoryByTime(start_t, end_t,partial_yaw_traj)) {
                return false;
            }

            return true;
        }

    };
}

#endif //EXP_TRAJ_H
