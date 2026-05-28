#ifndef YAML_LOADER_HPP
#define YAML_LOADER_HPP

#if __cplusplus < 201402L
#error "This code requires C++14 or higher. Please set the compiler to use C++14."
#endif


#include <iostream>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <stdexcept>
#include <vector>
#include <type_traits>

namespace yaml_loader {
    template <typename T>
    struct is_vector : std::false_type {};

    template <typename T, typename Alloc>
    struct is_vector<std::vector<T, Alloc>> : std::true_type {};


    class YamlLoader {
    public:
        explicit YamlLoader(const std::string& file_path) {
            path_ = file_path;
            std::cout << "Load config file: " << path_ << std::endl;
        }

        template <typename T>
        bool LoadParam(const std::string& param_name, T& param_value, const T& default_value = T{},
                       const bool& required = false) {
            return loadParamInternal(param_name, param_value, default_value, required);
        }

        template <class T>
        bool LoadParam(const std::string& param_name, std::vector<T>& param_value,
                       const std::vector<T> default_value = {}, const bool& required = false) {
            return loadParamInternal(param_name, param_value, default_value, required);
        }

    private:
        std::string path_;

        template <typename T>
        bool loadParamInternal(const std::string& param_name, T& param_value, const T& default_value, bool required) {
            const auto config_ = YAML::LoadFile(path_);
            YAML::Node tmp_node;
            if (containsSlash(param_name)) {
                tmp_node = getNodeFromPath(param_name);
            }
            else {
                if (config_[param_name]) {
                    tmp_node = config_[param_name];
                }
            }

            if (!tmp_node.IsNull()) {
                // 检查类型是否匹配
                if constexpr (is_vector<T>::value) {
                    if (tmp_node.Type() == YAML::NodeType::Sequence) {
                        try {
                            param_value = tmp_node.as<T>();
                            printf("\033[0;32m Load param %s success: \033[0;0m", param_name.c_str());
                            printValue(param_value);
                            return true;
                        } catch (const YAML::BadConversion&) {
                            printf("\033[0;31m Type mismatch for param %s, using default value: \033[0;0m", param_name.c_str());
                            param_value = default_value;
                            printValue(param_value);
                            return false;
                        }
                    } else {
                        printf("\033[0;31m Type mismatch for param %s, using default value: \033[0;0m", param_name.c_str());
                        param_value = default_value;
                        printValue(param_value);
                        return false;
                    }
                } else {
                    // 处理非 vector 类型
                    try {
                        param_value = tmp_node.as<T>();
                        printf("\033[0;32m Load param %s success: \033[0;0m", param_name.c_str());
                        printValue(param_value);
                        return true;
                    } catch (const YAML::BadConversion&) {
                        printf("\033[0;31m Type mismatch for param %s, using default value: \033[0;0m", param_name.c_str());
                        param_value = default_value;
                        printValue(param_value);
                        return false;
                    }
                }
            }
            else {
                printf("\033[0;33m Load param %s failed, use default value: \033[0;0m", param_name.c_str());
                param_value = default_value;
                printValue(param_value);
                if (required) {
                    throw std::invalid_argument("Required param " + param_name + " not found");
                }
                return false;
            }
        }

        static bool containsSlash(const std::string& str) {
            return str.find('/') != std::string::npos;
        }

        template <typename T>
        void printValue(const T& param_value) {
            if constexpr (is_vector<T>::value) {
                std::cout << "[";
                for (const auto& elem : param_value) {
                    std::cout << elem << " ";
                }
                std::cout << "]" << std::endl;
            }
            else {
                std::cout << param_value << std::endl;
            }
        }

        [[nodiscard]] YAML::Node getNodeFromPath(const std::string& path) const {
            auto node = YAML::LoadFile(path_);
            size_t pos = 0, next_pos;

            while ((next_pos = path.find('/', pos)) != std::string::npos) {
                std::string key = path.substr(pos, next_pos - pos);
                if (node[key]) {
                    node = node[key];
                }
                else {
                    return {};
                }
                pos = next_pos + 1;
            }
            if (node.IsScalar()) {
                return {};
            }
            if (!node[path.substr(pos)].IsNull() &&
                node[path.substr(pos)]) {
                node = node[path.substr(pos)];
                return node;
            }
            return {};
        }
    };
}


#endif //YAML_LOADER_HPP
