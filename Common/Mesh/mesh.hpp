#pragma once

#include <Eigen/Core>


namespace mesh{


    template <typename T>
    using MatrixX =  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

    template <typename T>
    using Vector3 = Eigen::Matrix<T, 3, 1>;

}