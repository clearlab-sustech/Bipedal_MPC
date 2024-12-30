#pragma once

#include <core/types.h>

#include <utility>

namespace clear {
class MatrixDB {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MatrixDB(std::string name = "default") : name_(name) {}

  MatrixDB(matrix_t A, vector_t b, matrix_t C, vector_t lb, vector_t ub,
           std::string name = "default")
      : A(std::move(A)),
        C(std::move(C)),
        b(std::move(b)),
        lb(std::move(lb)),
        ub(std::move(ub)),
        name_(name) {}

  MatrixDB operator+(const MatrixDB &rhs) const {
    return {concatenateMatrices(A, rhs.A),  concatenateVectors(b, rhs.b),
            concatenateMatrices(C, rhs.C),  concatenateVectors(lb, rhs.lb),
            concatenateVectors(ub, rhs.ub), name_ + rhs.name()};
  }

  MatrixDB operator*(scalar_t rhs) const {  // clang-format off
            return {A.cols() > 0 ? rhs * A : A,
                    b.cols() > 0 ? rhs * b : b,
                    C.cols() > 0 ? rhs * C : C,
                    lb.cols() > 0 ? rhs * lb : lb,
                    ub.cols() > 0 ? rhs * ub : ub, name_};  // clang-format on
  }

  MatrixDB operator*(matrix_t rhs) const {  // clang-format off
            return {A.cols() > 0 ? rhs * A : A,
                    b.cols() > 0 ? rhs * b : b,
                    C, lb, ub, name_};  // clang-format on
  }

  matrix_t A, C;
  vector_t b, lb, ub;

  matrix_t concatenateMatrices(matrix_t m1, matrix_t m2) const {
    if (m1.cols() <= 0) {
      return m2;
    } else if (m2.cols() <= 0) {
      return m1;
    }
    if (m1.cols() != m2.cols()) {
      throw std::runtime_error(
          name_ + "_MatrixDB: concatenateMatrices --> m1.cols() != m2.cols()");
    }
    matrix_t res(m1.rows() + m2.rows(), m1.cols());
    res << m1, m2;
    return res;
  }

  vector_t concatenateVectors(const vector_t &v1, const vector_t &v2) const {
    if (v1.rows() <= 0) {
      return v2;
    } else if (v2.rows() <= 0) {
      return v1;
    }
    vector_t res(v1.rows() + v2.rows());
    res << v1, v2;
    return res;
  }

  std::string name() const { return name_; }

 private:
  std::string name_;
};
}  // namespace clear