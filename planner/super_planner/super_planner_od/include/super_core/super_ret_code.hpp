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

#ifndef SUPER_RET_CODE_HPP
#define SUPER_RET_CODE_HPP

#include <cstring>
#include <vector>

namespace super_planner {
    enum SUPER_RET_CODE {
        SUPER_SUCCESS_WITH_BACKUP = 3,
        SUPER_SUCCESS_NO_BACKUP = 2,
        SUPER_SUCCESS = 1,
        SUPPER_UNDEFINED = -0,
        SUPER_NO_ODOM = -1,
        SUPER_NO_START_POINT = -2,

    };

    static std::string SUPER_RET_CODE_STR(const int& ret) {
        switch (ret) {
        case SUPER_SUCCESS_WITH_BACKUP:
            return "Success, with backup trajectory also success";
        case SUPER_SUCCESS_NO_BACKUP:
            return "Success, without need of backup";
        case SUPER_SUCCESS:
            return "Success";
        case SUPPER_UNDEFINED:
            return "Undefined";
        case SUPER_NO_ODOM:
            return "No odom, return at the start of the replan";
        case SUPER_NO_START_POINT:
            return "Cannot find a start point in the local map";
        }
    };
}
#endif
