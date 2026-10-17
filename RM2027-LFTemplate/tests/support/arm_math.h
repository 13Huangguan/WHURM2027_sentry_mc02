// Test-only scalar CMSIS-DSP substitute. Firmware uses the vendor ARM library.
#ifndef TEST_ARM_MATH_H
#define TEST_ARM_MATH_H

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>

#define PI 3.14159265358979323846f

enum arm_status {
    ARM_MATH_SUCCESS = 0,
    ARM_MATH_ARGUMENT_ERROR = -1,
    ARM_MATH_LENGTH_ERROR = -2,
    ARM_MATH_SIZE_MISMATCH = -3,
    ARM_MATH_SINGULAR = -5
};

struct arm_matrix_instance_f32 {
    uint16_t numRows;
    uint16_t numCols;
    float* pData;
};

inline void
arm_mat_init_f32(arm_matrix_instance_f32* view, uint16_t rows, uint16_t cols, float* data) {
    *view = { rows, cols, data };
}

inline arm_status arm_mat_add_f32(
    const arm_matrix_instance_f32* a,
    const arm_matrix_instance_f32* b,
    arm_matrix_instance_f32* output
) {
    for (uint32_t index = 0; index < uint32_t(a->numRows) * a->numCols; ++index) {
        output->pData[index] = a->pData[index] + b->pData[index];
    }
    return ARM_MATH_SUCCESS;
}

inline arm_status arm_mat_sub_f32(
    const arm_matrix_instance_f32* a,
    const arm_matrix_instance_f32* b,
    arm_matrix_instance_f32* output
) {
    for (uint32_t index = 0; index < uint32_t(a->numRows) * a->numCols; ++index) {
        output->pData[index] = a->pData[index] - b->pData[index];
    }
    return ARM_MATH_SUCCESS;
}

inline arm_status arm_mat_mult_f32(
    const arm_matrix_instance_f32* a,
    const arm_matrix_instance_f32* b,
    arm_matrix_instance_f32* output
) {
    for (uint16_t row = 0; row < a->numRows; ++row) {
        for (uint16_t col = 0; col < b->numCols; ++col) {
            float value = 0;
            for (uint16_t inner = 0; inner < a->numCols; ++inner) {
                value += a->pData[row * a->numCols + inner] * b->pData[inner * b->numCols + col];
            }
            output->pData[row * b->numCols + col] = value;
        }
    }
    return ARM_MATH_SUCCESS;
}

inline arm_status
arm_mat_trans_f32(const arm_matrix_instance_f32* input, arm_matrix_instance_f32* output) {
    for (uint16_t row = 0; row < input->numRows; ++row) {
        for (uint16_t col = 0; col < input->numCols; ++col) {
            output->pData[col * input->numRows + row] = input->pData[row * input->numCols + col];
        }
    }
    return ARM_MATH_SUCCESS;
}

inline arm_status arm_mat_scale_f32(
    const arm_matrix_instance_f32* input,
    float factor,
    arm_matrix_instance_f32* output
) {
    for (uint32_t index = 0; index < uint32_t(input->numRows) * input->numCols; ++index) {
        output->pData[index] = input->pData[index] * factor;
    }
    return ARM_MATH_SUCCESS;
}

// Like CMSIS, Gauss-Jordan inversion intentionally modifies the input buffer.
inline arm_status
arm_mat_inverse_f32(const arm_matrix_instance_f32* input, arm_matrix_instance_f32* output) {
    const int dim = input->numRows;
    auto* scratch = input->pData;
    std::fill_n(output->pData, dim * dim, 0.0f);
    for (int row = 0; row < dim; ++row) {
        output->pData[row * dim + row] = 1.0f;
    }
    for (int pivot = 0; pivot < dim; ++pivot) {
        int best_row = pivot;
        for (int row = pivot + 1; row < dim; ++row) {
            if (std::abs(scratch[row * dim + pivot]) > std::abs(scratch[best_row * dim + pivot])) {
                best_row = row;
            }
        }
        if (scratch[best_row * dim + pivot] == 0.0f) {
            return ARM_MATH_SINGULAR;
        }
        for (int col = 0; col < dim; ++col) {
            std::swap(scratch[pivot * dim + col], scratch[best_row * dim + col]);
            std::swap(output->pData[pivot * dim + col], output->pData[best_row * dim + col]);
        }
        const float factor = 1.0f / scratch[pivot * dim + pivot];
        for (int col = 0; col < dim; ++col) {
            scratch[pivot * dim + col] *= factor;
            output->pData[pivot * dim + col] *= factor;
        }
        for (int row = 0; row < dim; ++row) {
            if (row == pivot) {
                continue;
            }
            const float multiplier = scratch[row * dim + pivot];
            for (int col = 0; col < dim; ++col) {
                scratch[row * dim + col] -= multiplier * scratch[pivot * dim + col];
                output->pData[row * dim + col] -= multiplier * output->pData[pivot * dim + col];
            }
        }
    }
    return ARM_MATH_SUCCESS;
}

inline void arm_dot_prod_f32(const float* a, const float* b, uint32_t count, float* result) {
    *result = 0.0f;
    for (uint32_t index = 0; index < count; ++index) {
        *result += a[index] * b[index];
    }
}
inline float arm_sin_f32(float value) {
    return std::sin(value);
}
inline float arm_cos_f32(float value) {
    return std::cos(value);
}

#endif
