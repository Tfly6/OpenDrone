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

#ifndef SUPER_COLOR_TEXT_HPP
#define SUPER_COLOR_TEXT_HPP

#include <string>
namespace color_text{
    static const std::string RESET = "\033[0m";
    static const std::string BLACK = "\033[30m";             /* Black */
    static const std::string RED = "\033[31m";             /* Red */
    static const std::string GREEN = "\033[32m";             /* Green */
    static const std::string YELLOW = "\033[33m";             /* Yellow */
    static const std::string BLUE = "\033[34m";             /* Blue */
    static const std::string MAGENTA = "\033[35m";             /* Magenta */
    static const std::string CYAN = "\033[36m";             /* Cyan */
    static const std::string WHITE = "\033[37m";             /* White */
    static const std::string REDPURPLE = "\033[95m";             /* Red Purple */
    static const std::string BOLDBLACK = "\033[1m\033[30m";      /* Bold Black */
    static const std::string BOLDRED = "\033[1m\033[31m";      /* Bold Red */
    static const std::string BOLDGREEN = "\033[1m\033[32m";      /* Bold Green */
    static const std::string BOLDYELLOW = "\033[1m\033[33m";      /* Bold Yellow */
    static const std::string BOLDBLUE = "\033[1m\033[34m";      /* Bold Blue */
    static const std::string BOLDMAGENTA = "\033[1m\033[35m";      /* Bold Magenta */
    static const std::string BOLDCYAN = "\033[1m\033[36m";      /* Bold Cyan */
    static const std::string BOLDWHITE = "\033[1m\033[37m";      /* Bold White */
    static const std::string BOLDREDPURPLE = "\033[1m\033[95m";  /* Bold Red Purple */
}

#endif //SUPER_COLOR_TEXT_HPP
