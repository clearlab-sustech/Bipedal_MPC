
#pragma once
#include "core/trajectory/CubicSplineInterpolation.h"
#include "core/trajectory/TrajectoryBase.h"

namespace clear {
class CubicSplineTrajectory : public TrajectoryBase<vector_t, vector_t> {

public:
  CubicSplineTrajectory(size_t nDim,
                        CubicSplineInterpolation::SplineType type =
                            CubicSplineInterpolation::SplineType::cspline,
                        bool monotonic = false);

  ~CubicSplineTrajectory();

  void set_boundary(CubicSplineInterpolation::BoundaryType left,
                    vector_t left_value,
                    CubicSplineInterpolation::BoundaryType right,
                    vector_t right_value);

  void fit(std::vector<scalar_t> time, std::vector<vector_t> value);

  virtual vector_t evaluate(scalar_t time);

  virtual std::vector<vector_t> evaluate(std::vector<scalar_t> time_array);

  virtual vector_t derivative(scalar_t time, size_t n);

  virtual std::vector<vector_t> derivative(std::vector<scalar_t> time_array,
                                           size_t n);

private:
  size_t nDim_;
  CubicSplineInterpolation::SplineType spline_type_;
  bool monotonic_;
  std::vector<std::shared_ptr<CubicSplineInterpolation>> splines_array_;
};

} // namespace clear
