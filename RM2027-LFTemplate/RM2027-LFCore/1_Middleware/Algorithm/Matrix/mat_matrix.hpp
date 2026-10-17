// Matrix/vector calculations adapted from Team JiaoLong-SJTU (Spoon Guan, 2023).
// See LICENSE for the original MIT license. CMSIS-DSP supplies the kernels.
#ifndef MAT_MATRIX_HPP
#define MAT_MATRIX_HPP

#include <algorithm>
#include <cmath>
#include <cstring>

#include "arm_math.h"

namespace matrix {

// A view borrows contiguous row-major storage. It never allocates or copies data.
using View = arm_matrix_instance_f32;

inline void init(View* view, uint16_t rows, uint16_t cols, float* data) {
    arm_mat_init_f32(view, rows, cols, data);
}

namespace detail {

    template<int COLS>
    inline void
    multiplyVectorRows(uint16_t rows, const float* values, const float* vector, float* output) {
        for (uint16_t row = 0; row < rows; ++row) {
            float sum = 0.0f;
            // COLS is constant: the compiler can unroll this while preserving sum order.
            for (int col = 0; col < COLS; ++col) {
                sum += values[col] * vector[col];
            }
            output[row] = sum;
            values += COLS;
        }
    }

    inline arm_status multiplyVectorUnchecked(
        uint16_t rows,
        uint16_t cols,
        const float* values,
        const float* vector,
        float* output
    ) {
        switch (cols) {
            case 1:
                multiplyVectorRows<1>(rows, values, vector, output);
                break;
            case 2:
                multiplyVectorRows<2>(rows, values, vector, output);
                break;
            case 3:
                multiplyVectorRows<3>(rows, values, vector, output);
                break;
            case 4:
                multiplyVectorRows<4>(rows, values, vector, output);
                break;
            default: {
                const View lhs { rows, cols, const_cast<float*>(values) };
                const View rhs { cols, 1, const_cast<float*>(vector) };
                View result { rows, 1, output };
                return arm_mat_mult_f32(&lhs, &rhs, &result);
            }
        }
        return ARM_MATH_SUCCESS;
    }

} // namespace detail

// A contiguous matrix-vector product without constructing three descriptors per call.
inline arm_status multiplyVector(
    uint16_t rows,
    uint16_t cols,
    const float* values,
    const float* vector,
    float* output
) {
    if (rows == 0 || cols == 0) {
        return ARM_MATH_LENGTH_ERROR;
    }
    if (values == nullptr || vector == nullptr || output == nullptr) {
        return ARM_MATH_ARGUMENT_ERROR;
    }
    if (output == values || output == vector) {
        return ARM_MATH_SIZE_MISMATCH;
    }
    return detail::multiplyVectorUnchecked(rows, cols, values, vector, output);
}

inline arm_status add(const View* lhs, const View* rhs, View* output) {
    if (lhs->numRows != rhs->numRows || lhs->numCols != rhs->numCols
        || output->numRows != lhs->numRows || output->numCols != lhs->numCols)
    {
        return ARM_MATH_SIZE_MISMATCH;
    }
    return arm_mat_add_f32(lhs, rhs, output);
}

inline arm_status subtract(const View* lhs, const View* rhs, View* output) {
    if (lhs->numRows != rhs->numRows || lhs->numCols != rhs->numCols
        || output->numRows != lhs->numRows || output->numCols != lhs->numCols)
    {
        return ARM_MATH_SIZE_MISMATCH;
    }
    return arm_mat_sub_f32(lhs, rhs, output);
}

// CMSIS multiplication/transpose require a destination distinct from their input.
inline arm_status multiply(const View* lhs, const View* rhs, View* output) {
    if (lhs->numCols != rhs->numRows || output->numRows != lhs->numRows
        || output->numCols != rhs->numCols || output->pData == lhs->pData
        || output->pData == rhs->pData)
    {
        return ARM_MATH_SIZE_MISMATCH;
    }
    if (output->numRows == 0 || output->numCols == 0 || lhs->numCols == 0) {
        return ARM_MATH_LENGTH_ERROR;
    }
    if (rhs->numCols == 1) {
        return detail::multiplyVectorUnchecked(
            lhs->numRows,
            lhs->numCols,
            lhs->pData,
            rhs->pData,
            output->pData
        );
    }
    return arm_mat_mult_f32(lhs, rhs, output);
}

inline arm_status transpose(const View* input, View* output) {
    if (input->numRows != output->numCols || input->numCols != output->numRows
        || input->pData == output->pData)
    {
        return ARM_MATH_SIZE_MISMATCH;
    }
    return arm_mat_trans_f32(input, output);
}

inline arm_status scale(const View* input, float factor, View* output) {
    if (input->numRows != output->numRows || input->numCols != output->numCols) {
        return ARM_MATH_SIZE_MISMATCH;
    }
    return arm_mat_scale_f32(input, factor, output);
}

// CMSIS inversion destroys its input. Supply disposable scratch storage here.
inline arm_status inverse(View* scratch, View* output) {
    if (scratch->numRows != scratch->numCols || scratch->numRows != output->numRows
        || output->numRows != output->numCols || scratch->pData == output->pData)
    {
        return ARM_MATH_SIZE_MISMATCH;
    }
    return arm_mat_inverse_f32(scratch, output);
}

template<int ROWS, int COLS>
class Matrix {
    static_assert(ROWS > 0 && COLS > 0, "Matrix dimensions must be positive.");

    template<int, int>
    friend class Matrix;

    struct UninitializedTag {};

    explicit Matrix(UninitializedTag) {
        init(&this->arm_mat, ROWS, COLS, this->data_);
    }

public:
    Matrix(): Matrix(UninitializedTag {}) {
        std::fill_n(this->data_, ROWS * COLS, 0.0f);
    }

    explicit Matrix(const float* data): Matrix(UninitializedTag {}) {
        std::memcpy(this->data_, data, sizeof(this->data_));
    }

    Matrix(const Matrix& other): Matrix(other.data_) {}

    Matrix& operator=(const Matrix& other) {
        if (this != &other) {
            std::memcpy(this->data_, other.data_, sizeof(this->data_));
        }
        return *this;
    }

    static constexpr int rows() {
        return ROWS;
    }
    static constexpr int cols() {
        return COLS;
    }
    float* operator[](int row) {
        return this->data_ + row * COLS;
    }
    const float* operator[](int row) const {
        return this->data_ + row * COLS;
    }
    float* data() {
        return this->data_;
    }
    const float* data() const {
        return this->data_;
    }

    Matrix& operator+=(const Matrix& other) {
        add(&this->arm_mat, &other.arm_mat, &this->arm_mat);
        return *this;
    }
    Matrix& operator-=(const Matrix& other) {
        subtract(&this->arm_mat, &other.arm_mat, &this->arm_mat);
        return *this;
    }
    Matrix& operator*=(float factor) {
        scale(&this->arm_mat, factor, &this->arm_mat);
        return *this;
    }
    Matrix& operator/=(float factor) {
        return *this *= 1.0f / factor;
    }
    Matrix operator+(const Matrix& other) const {
        Matrix result(UninitializedTag {});
        add(&this->arm_mat, &other.arm_mat, &result.arm_mat);
        return result;
    }
    Matrix operator-(const Matrix& other) const {
        Matrix result(UninitializedTag {});
        subtract(&this->arm_mat, &other.arm_mat, &result.arm_mat);
        return result;
    }
    Matrix operator*(float factor) const {
        Matrix result(UninitializedTag {});
        scale(&this->arm_mat, factor, &result.arm_mat);
        return result;
    }
    Matrix operator/(float factor) const {
        return *this * (1.0f / factor);
    }
    friend Matrix operator*(float factor, const Matrix& value) {
        return value * factor;
    }

    template<int RESULT_COLS>
    Matrix<ROWS, RESULT_COLS> operator*(const Matrix<COLS, RESULT_COLS>& other) const {
        Matrix<ROWS, RESULT_COLS> result(typename Matrix<ROWS, RESULT_COLS>::UninitializedTag {});
        multiply(&this->arm_mat, &other.arm_mat, &result.arm_mat);
        return result;
    }

    template<int BLOCK_ROWS, int BLOCK_COLS>
    Matrix<BLOCK_ROWS, BLOCK_COLS> block(int start_row, int start_col) const {
        static_assert(BLOCK_ROWS <= ROWS && BLOCK_COLS <= COLS, "Block exceeds matrix.");
        if (start_row < 0 || start_col < 0 || start_row + BLOCK_ROWS > ROWS
            || start_col + BLOCK_COLS > COLS)
        {
            return Matrix<BLOCK_ROWS, BLOCK_COLS>();
        }
        Matrix<BLOCK_ROWS, BLOCK_COLS> result(
            typename Matrix<BLOCK_ROWS, BLOCK_COLS>::UninitializedTag {}
        );
        for (int row = 0; row < BLOCK_ROWS; ++row) {
            std::memcpy(
                result[row],
                (*this)[start_row + row] + start_col,
                BLOCK_COLS * sizeof(float)
            );
        }
        return result;
    }
    Matrix<1, COLS> row(int index) const {
        return this->template block<1, COLS>(index, 0);
    }
    Matrix<ROWS, 1> col(int index) const {
        return this->template block<ROWS, 1>(0, index);
    }
    Matrix<COLS, ROWS> trans() const {
        Matrix<COLS, ROWS> result(typename Matrix<COLS, ROWS>::UninitializedTag {});
        transpose(&this->arm_mat, &result.arm_mat);
        return result;
    }
    float trace() const {
        float result = 0.0f;
        for (int index = 0; index < std::min(ROWS, COLS); ++index) {
            result += (*this)[index][index];
        }
        return result;
    }
    float norm() const {
        float squared_norm;
        arm_dot_prod_f32(this->data_, this->data_, ROWS * COLS, &squared_norm);
        return std::sqrt(squared_norm);
    }

    View arm_mat;

private:
    float data_[ROWS * COLS];
};

template<int ROWS, int COLS>
Matrix<ROWS, COLS> zeros() {
    return Matrix<ROWS, COLS>();
}

template<int ROWS, int COLS>
Matrix<ROWS, COLS> ones() {
    Matrix<ROWS, COLS> result;
    std::fill_n(result.data(), ROWS * COLS, 1.0f);
    return result;
}

template<int ROWS, int COLS>
Matrix<ROWS, COLS> eye() {
    Matrix<ROWS, COLS> result;
    for (int index = 0; index < std::min(ROWS, COLS); ++index) {
        result[index][index] = 1.0f;
    }
    return result;
}

template<int ROWS, int COLS>
Matrix<ROWS, COLS> diag(const Matrix<ROWS, 1>& diagonal) {
    Matrix<ROWS, COLS> result;
    for (int index = 0; index < std::min(ROWS, COLS); ++index) {
        result[index][index] = diagonal[index][0];
    }
    return result;
}

// Preserve the caller's matrix and expose singularity to the caller.
template<int DIM>
arm_status inverse(const Matrix<DIM, DIM>& input, Matrix<DIM, DIM>& output) {
    Matrix<DIM, DIM> scratch(input);
    return inverse(&scratch.arm_mat, &output.arm_mat);
}

Matrix<3, 3> hat(const Matrix<3, 1>& vector);
Matrix<3, 1> cross(const Matrix<3, 1>& lhs, const Matrix<3, 1>& rhs);

} // namespace matrix

#endif // MAT_MATRIX_HPP
