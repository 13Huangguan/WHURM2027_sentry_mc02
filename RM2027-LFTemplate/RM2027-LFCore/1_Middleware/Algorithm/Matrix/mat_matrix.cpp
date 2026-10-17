#include "mat_matrix.hpp"

// Adapted from matrix.cpp, Copyright (c) 2023 Team JiaoLong-SJTU.
// Author: Spoon Guan. Distributed under the MIT license (see LICENSE).

namespace matrix {

Matrix<3, 3> hat(const Matrix<3, 1>& vector) {
    const float data[9] = { 0.0f,          -vector[2][0], vector[1][0], vector[2][0], 0.0f,
                            -vector[0][0], -vector[1][0], vector[0][0], 0.0f };
    return Matrix<3, 3>(data);
}

Matrix<3, 1> cross(const Matrix<3, 1>& lhs, const Matrix<3, 1>& rhs) {
    return hat(lhs) * rhs;
}

} // namespace matrix
