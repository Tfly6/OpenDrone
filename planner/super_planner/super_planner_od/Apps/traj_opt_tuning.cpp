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


#include <fsm/fsm.h>
#include <ncurses.h>
#include <filesystem>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <string>

#include <utils/header/yaml_loader.hpp>
#include <traj_opt/exp_traj_optimizer_s4.h>

using namespace super_utils;
using namespace fsm;

using PointType = pcl::PointXYZI;

namespace fs = std::filesystem;

std::string format_time(const std::filesystem::file_time_type& file_time) {
    auto sys_time = std::chrono::system_clock::from_time_t(
        std::chrono::duration_cast<std::chrono::seconds>(file_time.time_since_epoch()).count());
    std::time_t cftime = std::chrono::system_clock::to_time_t(sys_time);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&cftime), "%Y-%m-%d %H:%M:%S");
    return ss.str();
}

// 获取目录下最新修改的 .bin 文件
std::pair<std::string, std::string> getLatestBinFile(const std::string& directory) {
    std::string latestFile;
    std::filesystem::file_time_type latestTime;

    try {
        for (const auto& entry : fs::directory_iterator(directory)) {
            if (entry.is_regular_file() && entry.path().extension() == ".bin") {
                auto currentModTime = fs::last_write_time(entry);
                if (latestFile.empty() || currentModTime > latestTime) {
                    latestFile = entry.path().string();
                    latestTime = currentModTime;
                }
            }
        }

        if (latestFile.empty()) {
            throw std::runtime_error("No .bin files found in the directory.");
        }

        return {latestFile, format_time(latestTime)};
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return {"", ""};
    }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "fsm_node");
    ros::NodeHandle nh("~");
    const std::string directory = LOG_FILE_DIR("replan_logs/");
    auto [latestFile, latestTime] = getLatestBinFile(directory);

    if (!latestFile.empty()) {
        std::cout << "Latest .bin file: " << latestFile << std::endl;
    } else {
        std::cout << "No .bin file found." << std::endl;
        return 0;
    }
    ros_interface::RosInterface::Ptr ros_ptr = std::make_shared<ros_interface::Ros1Interface>(nh);
    vector<LogOneReplan> replan_logs;
    BinaryFileHandler<vector<LogOneReplan>>::load(latestFile, replan_logs);

    // read and plot
    sleep(5);
    ros::AsyncSpinner spinner(0);
    spinner.start();



    // enable keyboard
    initscr(); // 启动 ncurses 模式
    cbreak(); // 禁用行缓冲，立即传递按键
    noecho(); // 不在终端显示按键
    keypad(stdscr, TRUE); // 启用功能键支持（非必需，但推荐）
    nodelay(stdscr, TRUE); // 非阻塞模式，不等待输入

    std::cout<<"Check sim time"<<std::endl;
    while (ros::ok()) {
        bool use_sim_time;
        if (nh.getParam("/use_sim_time", use_sim_time)) {
            if (!use_sim_time)
            {
                fmt::print(" -- [Bench] Use sim time is false, begin replay.\n");
                break;
            }
            nh.setParam("/use_sim_time", false);
        } else{
            nh.setParam("/use_sim_time", false);
        }
        usleep(1e6);
    }

    fmt::print("===================== INFO =================.\n");
    fmt::print("Press key p to pause or run, press up or down to move.\n");
    int replan_id{0};
    bool pause{true};
    while (ros::ok()) {
        // if pause, wait for keyboard, else sleep 0.05
        int ch = getch(); // 获取按键
        if (ch == 'p') {
            pause = !pause;
        }

        if (pause) {
            usleep(1000);
            if (ch == KEY_UP || ch == 'u') {
                replan_id--;
                if (replan_id < 0) replan_id = 0;
            }
            else if (ch == KEY_DOWN || ch == 'd') {
                replan_id++;
                if (replan_id >= replan_logs.size()) replan_id = replan_logs.size() - 1;
            }else if (ch == 'r'){

            }
            else {
                continue;
            }
        }
        else {
            replan_id++;
            usleep(100000);
        }

        auto& replan = replan_logs[replan_id];
//        system("clear");
        fmt::print(" -- [Bench] Visualize replan ID: {}\n", replan_id);
        visualization_msgs::Marker del;
        visualization_msgs::MarkerArray arr;
        del.action = visualization_msgs::Marker::DELETEALL;
        arr.markers.push_back(del);
        ros_ptr->setVisualizationEn(true);

        // setup optimizer
#define CONFIG_FILE_DIR(name) (std::string(std::string(ROOT_DIR) + "config/"+(name)))
        const std::string path = CONFIG_FILE_DIR("click.yaml");
        auto cfg = traj_opt::Config(path,"exp_traj");
        auto b_cfg = traj_opt::Config(path,"backup_traj");
        ExpTrajOpt::Ptr exp_traj_opt = std::make_shared<ExpTrajOpt>(cfg,ros_ptr);
        BackupTrajOpt::Ptr backup_traj_opt = std::make_shared<BackupTrajOpt>(b_cfg,ros_ptr);
        Trajectory traj;
        replan.replanExpTrajectory(exp_traj_opt, traj);
        ros_ptr->vizExpTraj(traj);
        Trajectory backup_traj;
        replan.replanBackupTrajectory(backup_traj_opt, backup_traj);
        ros_ptr->vizBackupTraj(backup_traj);
        replan.visualize(ros_ptr);
        replan.print();
    }

    ros::waitForShutdown();
    return 0;
}
