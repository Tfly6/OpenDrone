//
// Created by yunfan on 11/8/24.
//

#ifndef CLASS_COLOR_MSG_UTILS
#define CLASS_COLOR_MSG_UTILS



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

