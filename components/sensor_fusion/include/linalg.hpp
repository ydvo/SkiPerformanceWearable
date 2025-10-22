#pragma once
#include <cmath>
#include <array>
#include <type_traits>

namespace linalg {

template<typename T, size_t Rows, size_t Cols>
inline constexpr std::array<T, Rows * Cols> matrix_identity() {
    std::array<T, Rows * Cols> result {};
    for (size_t i = 0; i < Rows && i < Cols; ++i) {
        result[i + Cols * i] = T{1}; 
    }

    return result; 
}

template<typename T, size_t Rows, size_t Cols>
inline constexpr void matrix_vector_multiply(
    const std::array<T, Rows * Cols>& matrix,
    const std::array<T, Cols>& vector,
    std::array<T, Rows>& result) noexcept {

    for (size_t i = 0; i < Rows; ++i) {
        T sum = T{};
        for (size_t j = 0; j < Cols; ++j) {
            sum += matrix[i * Cols + j] * vector[j];
        }
        result[i] = sum;
    }
}

template<typename T, size_t Rows, size_t Inner, size_t Cols>
inline constexpr void matrix_multiply(
    const std::array<T, Rows * Inner>& matrix1,
    const std::array<T, Inner * Cols>& matrix2,
    std::array<T, Rows * Cols>& result) noexcept {

    for (size_t i = 0; i < Rows; ++i) {
        for (size_t j = 0; j < Cols; ++j) {
            T sum = T{};
            for (size_t k = 0; k < Inner; ++k) {
                sum += matrix1[i * Inner + k] * matrix2[k * Cols + j];
            }
            result[i * Cols + j] = sum;
        }
    }
}

template<typename T, size_t Rows, size_t Cols>
inline constexpr void matrix_transpose(
    const std::array<T, Rows * Cols>& matrix,
    std::array<T, Cols * Rows>& result) noexcept {

    for (size_t i = 0; i < Rows; ++i) {
        for (size_t j = 0; j < Cols; ++j) {
            result[j * Rows + i] = matrix[i * Cols + j];
        }
    }
}

template<typename T, size_t Rows, size_t Cols>
inline constexpr void matrix_add(
    const std::array<T, Rows * Cols>& matrix1,
    const std::array<T, Rows * Cols>& matrix2,
    std::array<T, Rows * Cols>& result) noexcept {
    
    for (size_t i = 0; i < Rows * Cols; ++i) {
        result[i] = matrix1[i] + matrix2[i];
    }
}

template<typename T, size_t Rows, size_t Cols>
inline constexpr void matrix_subtract(
    const std::array<T, Rows * Cols>& matrix1,
    const std::array<T, Rows * Cols>& matrix2,
    std::array<T, Rows * Cols>& result) noexcept {
    
    for (size_t i = 0; i < Rows * Cols; ++i) {
        result[i] = matrix1[i] - matrix2[i];
    }
}

template<typename T>
inline constexpr int matrix_inverse_3x3(
    const std::array<T, 9>& matrix, 
    std::array<T, 9>& result) noexcept {
    
    T det = matrix[0] * (matrix[4] * matrix[8] - matrix[5] * matrix[7]) -
            matrix[1] * (matrix[3] * matrix[8] - matrix[5] * matrix[6]) +
            matrix[2] * (matrix[3] * matrix[7] - matrix[4] * matrix[6]);
    
    if (det == T{}) {
        return 1;
    }
    
    T inv_det = T{1} / det;
    
    result[0] = (matrix[4] * matrix[8] - matrix[7] * matrix[5]) * inv_det;
    result[1] = (matrix[2] * matrix[7] - matrix[1] * matrix[8]) * inv_det;
    result[2] = (matrix[1] * matrix[5] - matrix[2] * matrix[4]) * inv_det;
    result[3] = (matrix[5] * matrix[6] - matrix[3] * matrix[8]) * inv_det;
    result[4] = (matrix[0] * matrix[8] - matrix[2] * matrix[6]) * inv_det;
    result[5] = (matrix[3] * matrix[2] - matrix[0] * matrix[5]) * inv_det;
    result[6] = (matrix[3] * matrix[7] - matrix[6] * matrix[4]) * inv_det;
    result[7] = (matrix[6] * matrix[1] - matrix[0] * matrix[7]) * inv_det;
    result[8] = (matrix[0] * matrix[4] - matrix[3] * matrix[1]) * inv_det;
    return 0;
}

template<typename T, size_t N>
using Vector = std::array<T, N>;

template<typename T, size_t Rows, size_t Cols>
using Matrix = std::array<T, Rows * Cols>;
}