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



#include <ros_interface/ros2/fsm_ros2.hpp>

/*
 * Test code:
 *      roslaunch simulator test_env.launch
 * */
#define BACKWARD_HAS_DW 1

#include "utils/header/backward.hpp"

namespace backward {
    backward::SignalHandling sh;
}


using namespace fsm;
using namespace std;
FsmRos2::Ptr fsm_ptr;

#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

    auto node = std::make_shared<rclcpp::Node>("fsm_node");

    while (rclcpp::ok()) {
        bool use_sim_time;

        if (node->get_parameter("use_sim_time", use_sim_time)) {
            if (!use_sim_time) {
                std::cout << " -- [Bench] Use sim time is false, begin replay." << std::endl;
                break;
            } else {
                node->set_parameter(rclcpp::Parameter("use_sim_time", false));
            }
        } else {
            node->set_parameter(rclcpp::Parameter("use_sim_time", false));
        }

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }


#define CONFIG_FILE_DIR(name) (std::string(std::string(ROOT_DIR) + "config/"+(name)))
    auto fsm_ptr = std::make_shared<FsmRos2>();
    std::string cfg_path = "click.yaml";
    node->declare_parameter("config_name", cfg_path);
    node->get_parameter("config_name", cfg_path);
    cfg_path = CONFIG_FILE_DIR(cfg_path);
    fsm_ptr->init(node,cfg_path);

    RCLCPP_INFO(node->get_logger(), "\033[32m -- [Fsm-Test] Begin.\033[0m");

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
