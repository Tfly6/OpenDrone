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


#ifdef USE_ROS1
#ifndef SRC_ROS1_VISUALIZER_HPP
#define SRC_ROS1_VISUALIZER_HPP

#include "ros_interface/ros1/ros1_adapter.hpp"


namespace ros_interface {

    class Ros1Interface : public RosInterface {
    public:

        explicit Ros1Interface(const ros::NodeHandle &nh)
                : nh_(nh) {

            /*=============================FOR Planner========================================*/
            goal_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization/goal", 100);

            exp_traj_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization/exp_traj", 100);
            backup_traj_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization/backup_traj", 100);
            committed_traj_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization/committed_traj", 100);

            exp_sfcs_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization/exp_sfc", 100);
            backup_sfc_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization/backup_sfc", 100);

            guide_path_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization/frontend_path", 100);

            yaw_traj_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization/yaw_traj", 100);

            /*=============================FOR A* debug ========================================*/
            astar_mkr_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization/astar_debug", 100);

            /*=============================FOR A* debug ========================================*/
            ciri_mkr_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization/ciri_debug_mkr", 100);
            ciri_pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("visualization/ciri_debug_pc", 100);
            /*=============================FOR replan log ========================================*/
            replan_log_pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("visualization/replan_log_pc", 100);
            replan_log_mkr_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization/replan_log_mkr", 100);
        }


        /*=============================FOR ROS logger ========================================*/
        void debug(const std::string& msg) override { ROS_DEBUG("%s", msg.c_str()); }
        void info(const std::string& msg) override { ROS_INFO("%s", msg.c_str()); }
        void warn(const std::string& msg) override { ROS_WARN("%s", msg.c_str()); }
        void error(const std::string& msg) override { ROS_ERROR("%s", msg.c_str()); }
        void fatal(const std::string& msg) override { ROS_FATAL("%s", msg.c_str()); }

        double getSimTime() override {
            return ros::Time::now().toSec();
        }

        void getSimTime(int32_t &sec, uint32_t &nsec) override{
            ros::Time now = ros::Time::now();
            sec = now.sec;
            nsec = now.nsec;
        }

        void setSimTime(const double &sim_time) override {
            ros::Time::setNow(ros::Time(sim_time));
        }

        /*=============================FOR Planner========================================*/
        void vizExpTraj(const Trajectory &traj, const std::string & ns="exp_traj") override {
            if (!visualization_en_) {
                return;
            }
            if (exp_traj_pub_.getNumSubscribers() <= 0) {
                return;
            }
            if(traj.empty()){
                return;
            }
            Ros1Adapter::deleteAllMarkerArray(exp_traj_pub_);
            visualization_msgs::MarkerArray mkr_arr;
            Ros1Adapter::addTrajectoryToMarkerArray(mkr_arr, traj, ns, Color::Green(), 0.08, true, true);
            exp_traj_pub_.publish(mkr_arr);
        }

        void vizBackupTraj(const Trajectory &traj) {
            if (!visualization_en_) {
                return;
            }
            if (backup_traj_pub_.getNumSubscribers() <= 0) {
                return;
            }
            if(traj.empty()){
                return;
            }
            Ros1Adapter::deleteAllMarkerArray(backup_traj_pub_);
            visualization_msgs::MarkerArray mkr_arr;
            Ros1Adapter::addTrajectoryToMarkerArray(mkr_arr, traj, "backup_traj", Color::Green(), 0.08, true, false);
            Ros1Adapter::addPointToMarkerArray(mkr_arr,traj.getPos(0),  Color::Gray(),"backup_traj_start", 0.31);
            backup_traj_pub_.publish(mkr_arr);
        }

        void vizFrontendPath(const super_utils::vec_Vec3f &path) override {
            if (!visualization_en_) {
                return;
            }
            if(path.empty()){
                return;
            }
            Ros1Adapter::deleteAllMarkerArray(guide_path_pub_);
            visualization_msgs::MarkerArray mkr_arr;
            Ros1Adapter::addPathToMarkerArray(mkr_arr, path, Color::Pink(), "guide_path", 0.1, 0.05);
            guide_path_pub_.publish(mkr_arr);
        }

        void vizExpSfc(const PolytopeVec &sfcs) override {
            if (!visualization_en_) {
                return;
            }
            if (exp_sfcs_pub_.getNumSubscribers() <= 0) {
                return;
            }
            if(sfcs.empty()){
                return;
            }
            Ros1Adapter::deleteAllMarkerArray(exp_sfcs_pub_);
            visualization_msgs::MarkerArray mkr_arr;
            int color_num = sfcs.size();
            int color_id = 0;
            for (auto p: sfcs) {
                double color_ratio = 1.0 - (double) color_id / color_num;
                Vec3f color_mag = tinycolormap::GetColor(color_ratio, tinycolormap::ColormapType::Jet).ConvertToEigen();
                color_id++;
                Color c(color_mag[0], color_mag[1], color_mag[2]);
                Ros1Adapter::addPolytopeToMarkerArray(mkr_arr, p,
                                                      "exp_sfc", false,
                                                      Color::SteelBlue(), c,
                                                      Color::Orange(), 0.15,
                                                      resolution_ / 2);
            }
            exp_sfcs_pub_.publish(mkr_arr);
        }

        void vizBackupSfc(const Polytope &sfc) override {
            if (!visualization_en_) {
                return;
            }
            if (backup_sfc_pub_.getNumSubscribers() <= 0) {
                return;
            }
            Ros1Adapter::deleteAllMarkerArray(backup_sfc_pub_);
            visualization_msgs::MarkerArray mkr_arr;
            Ros1Adapter::addPolytopeToMarkerArray(mkr_arr, sfc, "backup_sfc", false, Color::Chartreuse(),
                                                  Color::Green(),
                                                  Color::Green(),
                                                  0.15,
                                                  resolution_ / 2);
            backup_sfc_pub_.publish(mkr_arr);
        }

        void vizGoalPath(const super_utils::vec_Vec3f &path) override {
            if (!visualization_en_) {
                return;
            }

            if (goal_pub_.getNumSubscribers() <= 0) {
                return;
            }

            if(path.empty()){
                return;
            }

            Ros1Adapter::deleteAllMarkerArray(goal_pub_);

            visualization_msgs::MarkerArray mkr_arr;
            Ros1Adapter::addPathToMarkerArray(mkr_arr, path, Color::Yellow(), "goal", 0.3, 0.15);
            goal_pub_.publish(mkr_arr);
        }

        void vizCommittedTraj(const geometry_utils::Trajectory &committed_traj,
                              const double &backup_traj_start_TT) override {
            if (!visualization_en_) {
                return;
            }
            if (committed_traj_pub_.getNumSubscribers() <= 0) {
                return;
            }

            if(committed_traj.empty()){
                return;
            }

            Ros1Adapter::deleteAllMarkerArray(committed_traj_pub_);

            visualization_msgs::MarkerArray mkr_arr;

            double traj_dur = committed_traj.getTotalDuration();

            if (backup_traj_start_TT > 0 && backup_traj_start_TT < traj_dur) {
                Trajectory exp_traj;
                if (!committed_traj.getPartialTrajectoryByTime(0, backup_traj_start_TT, exp_traj)) {
                    ROS_ERROR("Failed to get partial trajectory");
                    return;
                }
                Trajectory backup_traj;
                if (!committed_traj.getPartialTrajectoryByTime(backup_traj_start_TT, traj_dur, backup_traj)) {
                    ROS_ERROR("Failed to get partial trajectory");
                    return;
                }
                visualization_msgs::MarkerArray mkr_arr;
                Ros1Adapter::addTrajectoryToMarkerArray(mkr_arr, exp_traj, "committed_exp", Color::SteelBlue(), 0.08,
                                                        true, false);
                Ros1Adapter::addTrajectoryToMarkerArray(mkr_arr, backup_traj, "committed_backup", Color::Green(), 0.1,
                                                        false, false);
                committed_traj_pub_.publish(mkr_arr);
            } else {
                visualization_msgs::MarkerArray mkr_arr;
                Ros1Adapter::addTrajectoryToMarkerArray(mkr_arr, committed_traj, "committed_exp", Color::Green(), 0.1,
                                                        true, false);
                committed_traj_pub_.publish(mkr_arr);
            }

            committed_traj_pub_.publish(mkr_arr);
        }

        void vizYawTraj(const Trajectory &pos_traj, const Trajectory &yaw_traj) override {
            if (!visualization_en_ || pos_traj.empty() || yaw_traj.empty()) {
                return;
            }

            if (yaw_traj_pub_.getNumSubscribers() <= 0) {
                return;
            }

            if(pos_traj.empty() || yaw_traj.empty()){
                return;
            }

            Ros1Adapter::deleteAllMarkerArray(yaw_traj_pub_);
            visualization_msgs::MarkerArray mkr_arr;
            Ros1Adapter::addYawTrajectoryToMarkerArray(mkr_arr, pos_traj, yaw_traj);
            yaw_traj_pub_.publish(mkr_arr);
        }


        /*=============================FOR A* debug ========================================*/
        void vizAstarBoundingBox(const super_utils::Vec3f &bbox_min, const super_utils::Vec3f &bbox_max) override {
            if (!visualization_en_) {
                return;
            }

            if (astar_mkr_pub_.getNumSubscribers() <= 0) {
                return;
            }

            visualization_msgs::MarkerArray mkr_arr;
            Ros1Adapter::addBoundingBoxToMarkerArray(mkr_arr, bbox_min, bbox_max, "local_map",
                                                     Color::Chartreuse());
            astar_mkr_pub_.publish(mkr_arr);
        }

        void vizAstarPoints(const super_utils::Vec3f &position, const Color &c,
                            const std::string &ns = "none", const double &size = 0.1,
                            const int &id = 0) override {
            if (!visualization_en_) {
                return;
            }

            if (astar_mkr_pub_.getNumSubscribers() <= 0) {
                return;
            }

            visualization_msgs::MarkerArray mkr_arr;
            Ros1Adapter::addPointToMarkerArray(mkr_arr, position, c, ns, size);
            astar_mkr_pub_.publish(mkr_arr);
        }

        /*=============================FOR replan log ========================================*/
        void vizReplanLog(const Trajectory &exp_traj, const Trajectory &backup_traj,
                          const Trajectory &exp_yaw_traj, const Trajectory &backup_yaw_traj,
                          const PolytopeVec &exp_sfc, const Polytope &backup_sfc,
                          const vec_Vec3f &pc_for_sfc, const int &ret_code) override {
            if (!visualization_en_) {
                return;
            }

            if (replan_log_mkr_pub_.getNumSubscribers() <= 0) {
                return;
            }

            ros_interface::Ros1Adapter::deleteAllMarkerArray(replan_log_mkr_pub_);

            visualization_msgs::MarkerArray mkr_arr;
            ros_interface::Ros1Adapter::addTrajectoryToMarkerArray(mkr_arr, exp_traj, "exp_traj", Color::Orange(), 0.1,
                                                                   false, false);
            ros_interface::Ros1Adapter::addTrajectoryToMarkerArray(mkr_arr, backup_traj, "backup_traj", Color::Green(),
                                                                   0.1, false, false);
            ros_interface::Ros1Adapter::addYawTrajectoryToMarkerArray(mkr_arr, exp_traj, exp_yaw_traj, "exp_yaw_traj");
            ros_interface::Ros1Adapter::addYawTrajectoryToMarkerArray(mkr_arr, backup_traj, backup_yaw_traj,
                                                                      "backup_yaw_traj");


            int color_id = 0;
            int color_num = exp_sfc.size();

            for (auto p: exp_sfc) {
                double color_ratio = 1.0 - (double) color_id / color_num;
                Vec3f color_mag = tinycolormap::GetColor(color_ratio, tinycolormap::ColormapType::Jet).ConvertToEigen();
                color_id++;
                Color c(color_mag[0], color_mag[1], color_mag[2]);
                ros_interface::Ros1Adapter::addPolytopeToMarkerArray(mkr_arr, p, "exp_sfc", false, Color::SteelBlue(),
                                                                     c,
                                                                     Color::Orange(), 0.15,
                                                                     0.02);
            }

            ros_interface::Ros1Adapter::addPolytopeToMarkerArray(mkr_arr, backup_sfc, "backup_sfc", false,
                                                                 Color::Chartreuse(), Color::Green(),
                                                                 Color::Green(),
                                                                 0.15,
                                                                 0.02);

            replan_log_mkr_pub_.publish(mkr_arr);

            // for pc
            // covert vecVec3f to pclPc
            rog_map::PointCloud pc;
            for (auto p_e: pc_for_sfc) {
                rog_map::PclPoint p;
                p.x = p_e.x();
                p.y = p_e.y();
                p.z = p_e.z();
                pc.push_back(p);
            }
            sensor_msgs::PointCloud2 pc2;
            pcl::toROSMsg(pc, pc2);
            pc2.header.frame_id = "world";
            pc2.header.stamp = ros::Time::now();
            replan_log_pc_pub_.publish(pc2);

            fmt::print("\tResult: {}\n", super_planner::SUPER_RET_CODE_STR(ret_code));
        }

        void vizCiriSeedLine(const super_utils::Vec3f &a, const super_utils::Vec3f &b, const double &robot_r) override {
            if (!visualization_en_) {
                return;
            }
            if (ciri_mkr_pub_.getNumSubscribers() <= 0) {
                return;
            }
            visualization_msgs::MarkerArray mkr_arr;
            ros_interface::Ros1Adapter::addLineToMarkerArray(mkr_arr, a, b,
                                                             Color::Pink(), Color::Orange(), "seed_line",
                                                             robot_r * 2,
                                                             robot_r * 2);
            ciri_mkr_pub_.publish(mkr_arr);
        }

        void vizCiriEllipsoid(const geometry_utils::Ellipsoid &ellipsoid) override{
            if (!visualization_en_) {
                return;
            }
            if (ciri_mkr_pub_.getNumSubscribers() <= 0) {
                return;
            }
            visualization_msgs::MarkerArray mkr_arr;
            ros_interface::Ros1Adapter::addEllipsoidToMarkerArray(mkr_arr, ellipsoid, "ellipsoid", Color(Color::Orange(), 0.3));
            ciri_mkr_pub_.publish(mkr_arr);
        }

        void vizCiriInfeasiblePoint(const super_utils::Vec3f p) override{
            if (!visualization_en_) {
                return;
            }
            if (ciri_mkr_pub_.getNumSubscribers() <= 0) {
                return;
            }
            visualization_msgs::MarkerArray mkr_arr;
            ros_interface::Ros1Adapter::addPointToMarkerArray(mkr_arr, p, Color::Red(), "infeasible_pt", 0.1);
            ciri_mkr_pub_.publish(mkr_arr);
        }

        void vizCiriPolytope(const geometry_utils::Polytope &polytope, const std::string & ns) override{
            if (!visualization_en_) {
                return;
            }
            if (ciri_mkr_pub_.getNumSubscribers() <= 0) {
                return;
            }
            visualization_msgs::MarkerArray mkr_arr;
            ros_interface::Ros1Adapter::addPolytopeToMarkerArray(mkr_arr, polytope, ns, true,
                                                                 Color::Chartreuse(), Color::Green(),
                                                                 Color::Green(),
                                                                 0.15,
                                                                 0.02);
            ciri_mkr_pub_.publish(mkr_arr);
        }

        void vizCiriPointCloud(const vec_Vec3f & points) override {
            if (!visualization_en_) {
                return;
            }

            if (ciri_pc_pub_.getNumSubscribers() <= 0) {
                return;
            }

            sensor_msgs::PointCloud2 pc2;
            ros_interface::Ros1Adapter::addVecPointsToPointCloud2(points, pc2);
            ciri_pc_pub_.publish(pc2);
        }

    private:
        ros::NodeHandle nh_;
        // viz markers
        ros::Publisher goal_pub_, backup_sfc_pub_, backup_traj_pub_, committed_traj_pub_,
                receding_traj_pub_, exp_sfcs_pub_, point_pub_, fov_pub_,
                exp_traj_pub_, astar_pub_, receding_sfc_pub_, backup_traj_star_point_, yaw_traj_pub_, guide_path_pub_;

        ros::Publisher astar_mkr_pub_;

        ros::Publisher replan_log_mkr_pub_, replan_log_pc_pub_;

        ros::Publisher ciri_mkr_pub_, ciri_pc_pub_;


    };
}

#endif //SRC_ROS1_VISUALIZER_HPP
#endif //USE_ROS1
