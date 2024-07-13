#pragma once

#include <Eigen/Dense>

#include <array>
#include <vector>

enum MatrixIdx { X = 0, Y = 1, Z = 2, A = 3, B = 4, C = 5 };

/**
 * template <typename Scalar = double, int Size = 36,
          typename MatrixType = std::array<double, Size>>

 *
*/

template <typename Scalar, int Size, typename MatrixType>
class AdaptableMatrix {};

template <typename Scalar, int Size>
class AdaptableMatrix<Scalar, Size, std::array<Scalar, Size>> {
  using MatrixType = std::array<Scalar, Size>;
  MatrixType matrix_;
  static constexpr size_t row_length_ = sqrt(Size);

public:
  AdaptableMatrix() : matrix_() {}

  template <size_t RequiredSize>
  std::array<Scalar, RequiredSize>
  getMatrix(const std::vector<MatrixIdx> &states) {
    std::array<Scalar, RequiredSize> output;
    size_t row_length = sqrt(RequiredSize);
    size_t row = 0;
    for (auto &&state : states) {
      size_t column = 0;
      for (auto &&state : states) {
        output[(row * row_length) + column] =
            matrix_[(state + row_length_) + state];
        column++;
      }
      row++;
    }
    return output;
  }
};

template <>
class AdaptableMatrix<double, 6, Eigen::Matrix<double, 6, 6, Eigen::RowMajor>> {
  using Scalar = double;
  using MatrixType = Eigen::Matrix<double, 6, 6, Eigen::RowMajor>;
  MatrixType e_matrix_;

public:
  AdaptableMatrix(std::array<double, 36> array) {
    e_matrix_ = Eigen::Map<MatrixType>(array.data());
  }

  Eigen::MatrixX<Scalar> getMatrix(const std::vector<MatrixIdx> &states) {
    Eigen::MatrixX<Scalar> output(states.size(), states.size());
    size_t i = 0;
    for (auto &&state_i : states) {
      size_t j = 0;
      for (auto &&state_j : states) {
        output(i, j) = e_matrix_(state_i, state_j);
        j++;
      }
      i++;
    }
    return output;
  }  
};

template <typename Scalar, int RowSize = 6, int ColSize = 6 > class FlexiMatrix {
  using MatrixType = Eigen::Matrix<Scalar, RowSize, ColSize, Eigen::RowMajor>;
  MatrixType matrix_;


public:
  FlexiMatrix(std::array<Scalar, RowSize * ColSize> array) {
    matrix_.resize(RowSize, ColSize);
    matrix_ = Eigen::Map<MatrixType>(array.data());
  }

  Eigen::MatrixX<Scalar> getMatrix(const std::vector<MatrixIdx> &states) {
    Eigen::MatrixX<Scalar> output(states.size(), states.size());
    size_t i = 0;
    for (auto &&state_i : states) {
      size_t j = 0;
      for (auto &&state_j : states) {
        output(i, j) = matrix_(state_i, state_j);
        j++;
      }
      i++;
    }
    return output;
  }
};




typedef AdaptableMatrix<double, 36, std::array<double, 36>>
    AdaptableMatrix_Array;
typedef AdaptableMatrix<double, 6, Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>
    AdaptableMatrix_Eigen;