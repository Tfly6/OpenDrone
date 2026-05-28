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

#ifndef CLASS_COLOR_MSG_UTILS
#define CLASS_COLOR_MSG_UTILS


#ifndef USE_ROS1
#ifndef USE_ROS2
#error "Please define either USE_ROS1 or USE_ROS2, but not both."
#endif
#endif

#ifdef USE_ROS1
#ifdef USE_ROS2
#error "Cannot use both USE_ROS1 and USE_ROS2 at the same time. Please define only one."
#endif
#endif


#ifdef USE_ROS1

#include "std_msgs/ColorRGBA.h"

class Color : public std_msgs::ColorRGBA {
public:
    Color() : std_msgs::ColorRGBA() {}

    Color(int hex_color) {
        int _r = (hex_color >> 16) & 0xFF;
        int _g = (hex_color >> 8) & 0xFF;
        int _b = hex_color & 0xFF;
        r = static_cast<double>(_r) / 255.0;
        g = static_cast<double>(_g) / 255.0;
        b = static_cast<double>(_b) / 255.0;
    }

    Color(Color c, double alpha) {
        r = c.r;
        g = c.g;
        b = c.b;
        a = alpha;
    }

    Color(double red, double green, double blue) : Color(red, green, blue, 1.0) {
        r = red > 1.0 ? red / 255.0 : red;
        g = green > 1.0 ? green / 255.0 : green;
        b = blue > 1.0 ? blue / 255.0 : blue;
    }

    Color(double red, double green, double blue, double alpha) : Color() {
        r = red > 1.0 ? red / 255.0 : red;
        g = green > 1.0 ? green / 255.0 : green;
        b = blue > 1.0 ? blue / 255.0 : blue;
        a = alpha;
    }

    static const Color White() { return Color(1.0, 1.0, 1.0); }

    static const Color Black() { return Color(0.0, 0.0, 0.0); }

    static const Color Gray() { return Color(0.5, 0.5, 0.5); }

    static const Color Red() { return Color(1.0, 0.0, 0.0); }

    static const Color Green() { return Color(0.0, 0.96, 0.0); }

    static const Color Blue() { return Color(0.0, 0.0, 1.0); }

    static const Color SteelBlue() { return Color(0.4, 0.7, 1.0); }

    static const Color Yellow() { return Color(1.0, 1.0, 0.0); }

    static Color Orange() { return Color(1.0, 0.5, 0.0); }

    static const Color Purple() { return Color(0.5, 0.0, 1.0); }

    static const Color Chartreuse() { return Color(0.5, 1.0, 0.0); }

    static const Color Teal() { return Color(0.0, 1.0, 1.0); }

    static const Color Pink() { return Color(1.0, 0.0, 0.5); }
};
#endif

#ifdef USE_ROS2
#include <std_msgs/msg/color_rgba.hpp>

class Color : public std_msgs::msg::ColorRGBA {
public:
    Color() : std_msgs::msg::ColorRGBA() {}

    Color(int hex_color) {
        int _r = (hex_color >> 16) & 0xFF;
        int _g = (hex_color >> 8) & 0xFF;
        int _b = hex_color & 0xFF;
        r = static_cast<double>(_r) / 255.0;
        g = static_cast<double>(_g) / 255.0;
        b = static_cast<double>(_b) / 255.0;
    }

    Color(Color c, double alpha) {
        r = c.r;
        g = c.g;
        b = c.b;
        a = alpha;
    }

    Color(double red, double green, double blue) : Color(red, green, blue, 1.0) {
        r = red > 1.0 ? red / 255.0 : red;
        g = green > 1.0 ? green / 255.0 : green;
        b = blue > 1.0 ? blue / 255.0 : blue;
    }

    Color(double red, double green, double blue, double alpha) : Color() {
        r = red > 1.0 ? red / 255.0 : red;
        g = green > 1.0 ? green / 255.0 : green;
        b = blue > 1.0 ? blue / 255.0 : blue;
        a = alpha;
    }

    static const Color White() { return Color(1.0, 1.0, 1.0); }

    static const Color Black() { return Color(0.0, 0.0, 0.0); }

    static const Color Gray() { return Color(0.5, 0.5, 0.5); }

    static const Color Red() { return Color(1.0, 0.0, 0.0); }

    static const Color Green() { return Color(0.0, 0.96, 0.0); }

    static const Color Blue() { return Color(0.0, 0.0, 1.0); }

    static const Color SteelBlue() { return Color(0.4, 0.7, 1.0); }

    static const Color Yellow() { return Color(1.0, 1.0, 0.0); }

    static Color Orange() { return Color(1.0, 0.5, 0.0); }

    static const Color Purple() { return Color(0.5, 0.0, 1.0); }

    static const Color Chartreuse() { return Color(0.5, 1.0, 0.0); }

    static const Color Teal() { return Color(0.0, 1.0, 1.0); }

    static const Color Pink() { return Color(1.0, 0.0, 0.5); }
};

#endif

#endif
