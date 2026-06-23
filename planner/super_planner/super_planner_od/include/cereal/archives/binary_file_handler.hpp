// Created by yunfan on 10/29/24

#ifndef BINARY_FILE_HANDLER_HPP
#define BINARY_FILE_HANDLER_HPP

#include <cereal/types/eigen.hpp>
#include <cereal/archives/binary.hpp>
#include <cereal/types/string.hpp>
#include <fstream>
#include <iostream>
#include <string>
#include <ctime>
#include <iomanip>
// Use const std::string for color definitions with meaningful names
const std::string COLOR_RESET = "\033[0m";
const std::string COLOR_ERROR = "\033[31m";  // Red for errors
const std::string COLOR_SUCCESS = "\033[32m"; // Green for success

// Template utility class for binary file operations
template <typename T>
class BinaryFileHandler {
public:

    static std::string getCurrentTimeStr() {
        // 获取当前时间
        std::time_t now = std::time(nullptr);
        std::tm localTime = *std::localtime(&now);

        // 使用 stringstream 格式化时间为 YYMMDD-HHMMSS
        std::ostringstream oss;
        oss << std::setfill('0') << std::setw(2) << (localTime.tm_year % 100)  // 年份后两位
            << std::setw(2) << (localTime.tm_mon + 1)                         // 月份 (tm_mon 从 0 开始)
            << std::setw(2) << localTime.tm_mday                              // 日期
            << "-"                                                            // 分隔符
            << std::setw(2) << localTime.tm_hour                              // 小时
            << std::setw(2) << localTime.tm_min                               // 分钟
            << std::setw(2) << localTime.tm_sec;                              // 秒

        return oss.str();
    }
    // Static method: Save data to a binary file
    static void save(const std::string& filename, const T& data) {
        std::ofstream ofs(filename, std::ios::binary);
        if (!ofs) {
            std::cerr << COLOR_ERROR << "Error: Could not open file for writing.\n"
                      << COLOR_RESET;
            return;
        }
        cereal::BinaryOutputArchive archive(ofs);
        archive(data);
        std::cout << COLOR_SUCCESS << "Data saved successfully to " << filename
                  << COLOR_RESET << '\n';
    }

    // Static method: Load data from a binary file
    static void load(const std::string& filename, T& data) {
        std::ifstream ifs(filename, std::ios::binary);
        if (!ifs) {
            std::cerr << COLOR_ERROR << "Error: Could not open file for reading.\n"
                      << COLOR_RESET;
            return;
        }
        cereal::BinaryInputArchive archive(ifs);
        archive(data);
        std::cout << COLOR_SUCCESS << "Data loaded successfully from " << filename
                  << COLOR_RESET << '\n';
    }
};

#endif // BINARY_FILE_HANDLER_HPP
