//
// Created by yunfan on 11/8/24.
//

#ifndef SUPER_TYPE_UTILS_HPP
#define SUPER_TYPE_UTILS_HPP

// comment or uncomment the following line to enable or disable the utils
#include <super_utils/color_text.hpp>
#include <super_utils/eigen_alias.hpp>

#define DEBUG_FILE_DIR(name) (string(string(ROOT_DIR) + "log/"+name))
#define PCD_FILE_DIR(name) (string(string(ROOT_DIR) + "pcd/"+name))
#define SIGN(x) ((x > 0) - (x < 0))

namespace super_utils{

    using std::vector;
    using std::string;
    using std::cout;
    using std::endl;


    enum RET_CODE {
        /// FOR Planner
        FAILED = 0,
        NO_NEED = 1,
        SUCCESS = 2,
        FINISH = 3,
        NEW_TRAJ = 4,
        EMER = 5,
        OPT_FAILED = 6,
        INIT_ERROR = 7,

        /// FOR path search
        REACH_HORIZON,
        REACH_GOAL,
        NO_PATH,
        TIME_OUT
    };

    static const std::vector<std::string> RET_CODE_STR{"FAILED", "NO_NEED", "SUCCESS",
                                                       "FINISH", "NEW_TRAJ", "EMER",
                                                       "OPT_FAILED","INIT_ERROR",
                                                        "REACH_HORIZON", "REACH_GOAL", "NO_PATH", "TIME_OUT"};

    enum GridType {
        UNDEFINED = 0,
        UNKNOWN = 1,
        OUT_OF_MAP,
        OCCUPIED,
        KNOWN_FREE,
        FRONTIER, // The frontier is an unknown grid which is adjacent to the known free grid
    };

    const static std::vector<std::string> GridTypeStr{"UNDEFINED",
                                                      "UNKNOWN",
                                                      "OUT_OF_MAP",
                                                      "OCCUPIED",
                                                      "KNOWN_FREE",
                                                      "FRONTIER"};

    template<typename T>
    std::ostream &operator<<(std::ostream &out, const std::vector<T> &v) {
        out << "[";
        for (typename std::vector<T>::const_iterator it = v.begin(); it != v.end(); ++it) {
            out << *it;
            if (it != v.end() - 1) {
                out << ", ";
            }
        }
        out << "]";
        return out;
    }

    struct RobotState {
        Vec3f p, v, a, j;
        double yaw;
        double rcv_time;
        bool rcv{false};
        Quatf q;
    };




}


#endif //SUPER_TYPE_UTILS_HPP
