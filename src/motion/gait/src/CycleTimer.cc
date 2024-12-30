#include "gait/CycleTimer.h"
#include <core/misc/NumericTraits.h>

namespace clear {
CycleTimer::CycleTimer(Node::SharedPtr nodeHandle, scalar_t cycle_duration)
    : nodeHandle_(nodeHandle), cycle_duration_(cycle_duration) {
  if (cycle_duration_ < numeric_traits::limitEpsilon<scalar_t>()) {
    throw std::out_of_range("cycle_duration less than 0");
  }
  cycle_start_point_.push(nodeHandle_->now().seconds());
  current_cycle_time_.push(0.0);
  inner_loop_thread_ = std::thread(&CycleTimer::innerLoop, this);
  run_.push(true);
}

CycleTimer::~CycleTimer() {
  run_.push(false);
  inner_loop_thread_.join();
}

void CycleTimer::innerLoop() {
  rclcpp::Rate loop_rate(10000.0);
  while (rclcpp::ok() && run_.get()) {
    while (cycle_start_point_.get() <
           nodeHandle_->now().seconds() - cycle_duration_) {
      cycle_start_point_.push(cycle_start_point_.get() + cycle_duration_);
    }
    if (cycle_start_point_.get() > nodeHandle_->now().seconds()) {
      cycle_start_point_.push(nodeHandle_->now().seconds());
    }
    current_cycle_time_.push(nodeHandle_->now().seconds() -
                             cycle_start_point_.get());
    loop_rate.sleep();
  }
}

void CycleTimer::timerReset() {
  cycle_start_point_.push(nodeHandle_->now().seconds());
}

scalar_t CycleTimer::getCycleTime() { return current_cycle_time_.get(); }

} // namespace clear
