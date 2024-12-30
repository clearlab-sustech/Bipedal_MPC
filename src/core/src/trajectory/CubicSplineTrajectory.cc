
#include "core/trajectory/CubicSplineTrajectory.h"
#include <rcpputils/asserts.hpp>

namespace clear {

CubicSplineTrajectory::CubicSplineTrajectory(
    size_t nDim, CubicSplineInterpolation::SplineType type, bool monotonic)
    : nDim_(nDim), spline_type_(type), monotonic_(monotonic) {}

CubicSplineTrajectory::~CubicSplineTrajectory() {}

void CubicSplineTrajectory::set_boundary(
    CubicSplineInterpolation::BoundaryType left, vector_t left_value,
    CubicSplineInterpolation::BoundaryType right, vector_t right_value) {
  rcpputils::assert_true(left_value.size() == static_cast<int>(nDim_) &&
                         right_value.size() == static_cast<int>(nDim_));
  if (splines_array_.size() != nDim_) {
    splines_array_.clear();
    for (size_t n = 0; n < nDim_; n++) {
      splines_array_.emplace_back(
          std::make_shared<CubicSplineInterpolation>(spline_type_, monotonic_));
      splines_array_[n]->set_boundary(left, left_value[n], right,
                                      right_value[n]);
    }
  }
}

void CubicSplineTrajectory::fit(std::vector<scalar_t> time,
                                std::vector<vector_t> value) {
  rcpputils::assert_true(
      std::all_of(value.begin(), value.end(), [this](vector_t &val) {
        return val.size() == static_cast<int>(this->nDim_);
      }));
  if (splines_array_.size() != nDim_) {
    splines_array_.clear();
    for (size_t n = 0; n < nDim_; n++) {
      splines_array_.emplace_back(
          std::make_shared<CubicSplineInterpolation>(spline_type_, monotonic_));
    }
  }
  for (size_t n = 0; n < nDim_; n++) {
    std::vector<scalar_t> y;
    for (size_t i = 0; i < value.size(); i++) {
      y.emplace_back(value[i](n));
    }
    splines_array_[n]->set_points(time, y);
  }
  TrajectoryBase::setTimeInterval(time.front(), time.back());
}

vector_t CubicSplineTrajectory::evaluate(scalar_t time) {
  rcpputils::assert_true(splines_array_.size() == nDim_);
  time = std::min(std::max(ts_, time), tf_);
  vector_t sample = vector_t::Zero(nDim_);
  for (size_t n = 0; n < nDim_; n++) {
    sample[n] = splines_array_[n]->evaluate(time);
  }
  return sample;
}

std::vector<vector_t>
CubicSplineTrajectory::evaluate(std::vector<scalar_t> time_array) {
  vector_array_t samples;
  for (const scalar_t &t : time_array) {
    samples.emplace_back(evaluate(t));
  }
  return samples;
}

vector_t CubicSplineTrajectory::derivative(scalar_t time, size_t nOrder) {
  rcpputils::assert_true(splines_array_.size() == nDim_);
  time = std::min(std::max(ts_, time), tf_);
  vector_t sample = vector_t::Zero(nDim_);
  for (size_t n = 0; n < nDim_; n++) {
    sample[n] = splines_array_[n]->derivative(time, nOrder);
  }
  return sample;
}

std::vector<vector_t>
CubicSplineTrajectory::derivative(std::vector<scalar_t> time_array, size_t n) {
  vector_array_t derivatives_val;
  for (const scalar_t &t : time_array) {
    derivatives_val.emplace_back(derivative(t, n));
  }
  return derivatives_val;
}

} // namespace clear
