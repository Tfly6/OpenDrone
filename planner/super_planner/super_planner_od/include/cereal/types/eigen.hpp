#ifndef EIGEN_CEREAL_HPP
#define EIGEN_CEREAL_HPP

#include <cereal/cereal.hpp>
#include <cereal/types/vector.hpp>
#include <Eigen/Dense>

namespace cereal {

    // 保存函数 (Output Serialization)
    template <class Archive, class _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
    inline typename std::enable_if<traits::is_output_serializable<BinaryData<_Scalar>, Archive>::value, void>::type
    save(Archive & ar, Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> const & m) {
        int32_t rows = m.rows();
        int32_t cols = m.cols();
        ar(rows);
        ar(cols);
        ar(binary_data(m.data(), rows * cols * sizeof(_Scalar)));
    }

    // 加载函数 (Input Serialization)
    template <class Archive, class _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
    inline typename std::enable_if<traits::is_input_serializable<BinaryData<_Scalar>, Archive>::value, void>::type
    load(Archive & ar, Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> & m) {
        int32_t rows;
        int32_t cols;
        ar(rows);
        ar(cols);
        m.resize(rows, cols);
        ar(binary_data(m.data(), static_cast<std::size_t>(rows * cols * sizeof(_Scalar))));
    }
}

#endif // EIGEN_CEREAL_HPP
