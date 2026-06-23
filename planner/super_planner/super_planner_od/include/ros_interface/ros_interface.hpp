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


#ifndef SRC_VISUALIZER_INTERFACE_HPP
#define SRC_VISUALIZER_INTERFACE_HPP

#include <memory>
#include <utils/header/eigen_alias.hpp>
#include <utils/header/color_msg_utils.hpp>
#include <fmt/color.h>
#include <data_structure/base/trajectory.h>
#include <data_structure/base/polytope.h>
#include <super_core/super_ret_code.hpp>

#define TINYCOLORMAP_WITH_EIGEN
#include "utils/header/tinycolormap.hpp"

namespace ros_interface{
    using namespace geometry_utils;
    using namespace super_utils;

    class RosInterface {
    protected:
        bool visualization_en_{false};
        double resolution_{0.1};
    public:
        using Ptr = std::shared_ptr<RosInterface>;

        virtual ~RosInterface() = default;
        /*===================For logger interface =======================*/
        template<typename... Args>
        static std::string format(const char* fmt, Args&&... args) {
            return fmt::format(fmt, std::forward<Args>(args)...);
        }
        // 核心日志接口
        virtual void debug(const std::string& msg) = 0;
        virtual void info(const std::string& msg) = 0;
        virtual void warn(const std::string& msg) = 0;
        virtual void error(const std::string& msg) = 0;
        virtual void fatal(const std::string& msg) = 0;

        // 格式化工具函数
        template<typename... Args>
        void debug(const char* fmt, Args... args) {
            debug(format(fmt, args...));
        }

        template<typename... Args>
        void info(const char* fmt, Args... args) {
            info(format(fmt, args...));
        }

        template<typename... Args>
        void warn(const char* fmt, Args... args) {
            warn(format(fmt, args...));
        }

        template<typename... Args>
        void error(const char* fmt, Args... args) {
            error(format(fmt, args...));
        }

        template<typename... Args>
        void fatal(const char* fmt, Args... args) {
            fatal(format(fmt, args...));
        }


        /*===================For time interface =======================*/
        virtual void setSimTime(const double &sim_time) = 0;

        virtual double getSimTime() = 0;

        virtual void getSimTime(int32_t &sec, uint32_t &nsec) = 0;

        /*===================For viz interface =======================*/
        virtual void vizExpTraj(const Trajectory &traj, const std::string & ns="exp_traj") = 0;

        virtual void vizBackupTraj(const Trajectory & traj) = 0;

        virtual void vizFrontendPath(const vec_Vec3f & path) = 0;

        virtual void vizExpSfc(const PolytopeVec & sfc) = 0;

        virtual void vizBackupSfc(const Polytope & sfc) = 0;

        virtual void vizGoalPath(const vec_Vec3f & path) = 0;

        virtual void vizCommittedTraj(const Trajectory &committed_traj, const double & backup_traj_start_TT) = 0;

        virtual void vizYawTraj(const Trajectory & pos_traj, const Trajectory & yaw_traj) = 0;

        /*For Astar debug ==================================*/
        virtual void vizAstarBoundingBox(const Vec3f & bbox_min, const Vec3f & bbox_max) = 0;

        virtual void vizAstarPoints(const Vec3f & position, const Color & c, const std::string & ns,
                                    const double & size = 0.1, const int & id = 0) = 0;

        /*For replan log==================================*/
        virtual void vizReplanLog(const Trajectory & exp_traj, const Trajectory & backup_traj,
                                  const Trajectory & exp_yaw_traj, const Trajectory & backup_yaw_traj,
                                  const PolytopeVec & exp_sfc, const Polytope & backup_sfc,
                                  const vec_Vec3f & pc_for_sfc, const int & ret_code) = 0;

        /*For CIRI debug==================================*/
        virtual void vizCiriSeedLine(const Vec3f & a, const Vec3f & b, const double & robot_r) = 0;

        virtual void vizCiriEllipsoid(const Ellipsoid & ellipsoid) = 0;

        virtual void vizCiriInfeasiblePoint(const Vec3f p) = 0;

        virtual void vizCiriPolytope(const Polytope & polytope, const std::string & ns) = 0;

        virtual void vizCiriPointCloud(const vec_Vec3f & points) = 0;


        /*Set some parameters==================================*/
        virtual void setResolution(const double &resolution) {
            resolution_ = resolution;
            fmt::print(fg(fmt::color::lime_green)," -- [Viz] Set resolution_ to {}\n", resolution_);
        }

        virtual void setVisualizationEn(const bool &  en) {
            visualization_en_ = en;
            fmt::print(fg(fmt::color::lime_green)," -- [Viz] Set visualization_en_ to {}\n", en);
        }

    };
}


#endif //SRC_VISUALIZER_INTERFACE_HPP
