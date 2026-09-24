#ifndef PRINTF_UTILS_H
#define PRINTF_UTILS_H

#include <string>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <ctime>
using namespace std;

#define NUM_POINT 2

#define RED "\033[0;1;31m"
#define GREEN "\033[0;1;32m"
#define YELLOW "\033[0;1;33m"
#define BLUE "\033[0;1;34m"
#define PURPLE "\033[0;1;35m"
#define DEEPGREEN "\033[0;1;36m"
#define WHITE "\033[0;1;37m"

#define RED_IN_WHITE "\033[0;47;31m"
#define GREEN_IN_WHITE "\033[0;47;32m"
#define YELLOW_IN_WHITE "\033[0;47;33m"

#define TAIL "\033[0m"

class Print
{
public:
    Print(float interval = 0, std::string color = TAIL)
        : interval(interval), past_ts(std::chrono::system_clock::now()), color(color)
    {
        std::cout.setf(ios::fixed);
        std::cout << std::setprecision(2);
        std::cout.setf(ios::left);
        std::cout.setf(ios::showpoint);
        std::cout.setf(ios::showpos);
    };

    template <typename T>
    void operator()(T &&msg)
    {
        std::chrono::system_clock::time_point now_ts = std::chrono::system_clock::now();
        auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(now_ts - past_ts).count();

        if (this != s_object_name)
        {
            std::cout << std::endl;
            s_object_name = this;
        }

        if (interval >= 0)
        {
            if (dt < this->interval * 1000)
                return;
            std::cout << color << msg << TAIL << std::endl;
        }
        else
        {
            if (dt < 0.1 * 1000)
                return;
            char now_char;
            switch (times)
            {
            case 0:
                now_char = '\\';
                break;
            case 1:
                now_char = '|';
                break;
            case 2:
                now_char = '/';
                break;
            case 3:
                now_char = '-';
                break;
            }
            times = ++times % 4;
            std::cout << color << "\r " << now_char << " " << msg << TAIL << std::flush;
        }
        this->past_ts = now_ts;
    };

    float interval;

private:
    std::chrono::system_clock::time_point past_ts;
    std::string color;
    static void *s_object_name;
    unsigned int times = 0;
};

void *Print::s_object_name = nullptr;

#define PRINTF_UTILS_CONCAT_(x, y) x##y
#define PRINTF_UTILS_CONCAT(x, y) PRINTF_UTILS_CONCAT_(x, y)

#define PRINTF_UTILS_PCOUT_(interval, color, msg, sign)             \
    static Print PRINTF_UTILS_CONCAT(print, sign)(interval, color); \
    PRINTF_UTILS_CONCAT(print, sign)                                \
    (msg)

#define PCOUT(interval, color, msg) PRINTF_UTILS_PCOUT_(interval, color, msg, __LINE__)

// Example:
// cout << GREEN << "Test for Green text." << TAIL <<endl;




#endif
