#pragma once

#include "core/types.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

namespace clear {
class CubicSplineInterpolation {
public:
  // boundary condition type for the spline end-points
  enum BoundaryType { first_deriv = 1, second_deriv = 2, not_a_knot = 3 };

  // spline types
  enum SplineType {
    linear = 10,         // linear interpolation
    cspline = 30,        // cubic splines (classical C^2)
    cspline_hermite = 31 // cubic hermite splines (local, only C^1)
  };

public:
  CubicSplineInterpolation(SplineType type = SplineType::cspline,
                           bool monotonic = false);

  ~CubicSplineInterpolation();

  void set_points(const std::vector<scalar_t> &x,
                  const std::vector<scalar_t> &y);

  void set_boundary(BoundaryType left, scalar_t left_value, BoundaryType right,
                    scalar_t right_value);

  scalar_t evaluate(scalar_t x) const;

  scalar_t derivative(scalar_t x, size_t order) const;

  std::string info() const;

protected:
  bool make_monotonic();

  void set_coeffs_from_b(); // calculate c_i, d_i from b_i

  size_t find_closest(scalar_t x) const; // closest idx so that m_x[idx]<=x

  // solves for all x so that: spline(x) = y
  std::vector<double> solve(double y, bool ignore_extrapolation = true) const;

private:
  bool isValid(const std::vector<scalar_t> &x,
               const std::vector<scalar_t> &y) const;

protected:
  std::vector<scalar_t> m_x, m_y; // x,y coordinates of points
  // interpolation parameters
  // f(x) = a_i + b_i*(x-x_i) + c_i*(x-x_i)^2 + d_i*(x-x_i)^3
  // where a_i = y_i, or else it won't go through grid points
  std::vector<scalar_t> m_b, m_c, m_d; // spline coefficients
  scalar_t m_c0;                       // for left extrapolation
  const SplineType m_type;
  BoundaryType m_left, m_right;
  scalar_t m_left_value, m_right_value;
  const bool m_made_monotonic;
};

namespace internal {
// band matrix solver
class BandMatrix {
private:
  std::vector<std::vector<double>> m_upper; // upper band
  std::vector<std::vector<double>> m_lower; // lower band
public:
  BandMatrix(){};                         // constructor
  BandMatrix(int dim, int n_u, int n_l);  // constructor
  ~BandMatrix(){};                        // destructor
  void resize(int dim, int n_u, int n_l); // init with dim,n_u,n_l
  int dim() const;                        // matrix dimension
  int num_upper() const { return (int)m_upper.size() - 1; }
  int num_lower() const { return (int)m_lower.size() - 1; }
  // access operator
  double &operator()(int i, int j);      // write
  double operator()(int i, int j) const; // read
  // we can store an additional diagonal (in m_lower)
  double &saved_diag(int i);
  double saved_diag(int i) const;
  void lu_decompose();
  std::vector<double> r_solve(const std::vector<double> &b) const;
  std::vector<double> l_solve(const std::vector<double> &b) const;
  std::vector<double> lu_solve(const std::vector<double> &b,
                               bool is_lu_decomposed = false);
};

double get_eps();

std::vector<double> solve_cubic(double a, double b, double c, double d,
                                int newton_iter = 0);
} // namespace internal

} // namespace clear
