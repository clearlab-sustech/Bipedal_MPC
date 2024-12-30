#pragma once

#include <core/misc/Buffer.h>
#include <core/types.h>
#include <rclcpp/rclcpp.hpp>

using namespace rclcpp;

namespace clear {
class CycleTimer {
public:
  CycleTimer(Node::SharedPtr nodeHandle, scalar_t cycle_duration);

  ~CycleTimer();

  void timerReset();

  scalar_t getCycleTime();

private:
  void innerLoop();

private:
  Node::SharedPtr nodeHandle_;
  scalar_t cycle_duration_;
  Buffer<scalar_t> cycle_start_point_;
  Buffer<scalar_t> current_cycle_time_;

  std::thread inner_loop_thread_;
  Buffer<bool> run_;
};

} // namespace clear
