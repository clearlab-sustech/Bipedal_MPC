#pragma once

#include "core/types.h"

namespace clear {

template <typename Sample, typename SampleDerivative> class TrajectoryBase {
public:
  TrajectoryBase() = default;

  virtual ~TrajectoryBase() = default;

  virtual Sample evaluate(scalar_t time) = 0;

  virtual std::vector<Sample> evaluate(std::vector<scalar_t> time_array) = 0;

  virtual SampleDerivative derivative(scalar_t time, size_t n) = 0;

  virtual std::vector<SampleDerivative>
  derivative(std::vector<scalar_t> time_array, size_t n) = 0;

  void setTimeInterval(scalar_t ts, scalar_t tf) {
    ts_ = ts;
    tf_ = ts <= tf ? tf : ts;
  }

  scalar_t ts() { return ts_; }

  scalar_t tf() { return tf_; }

  scalar_t duration() { return std::max(tf_ - ts_, 0.0); }

protected:
  scalar_t ts_, tf_;
};

} // namespace clear
