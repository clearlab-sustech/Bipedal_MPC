#pragma once

#include "trans/srv/simulation_reset.hpp"
#include <core/types.h>
#include <rclcpp/rclcpp.hpp>

using namespace rclcpp;
using namespace std::chrono_literals;

namespace clear {
class Initialization {

public:
  Initialization(Node::SharedPtr nodeHandle);

  ~Initialization();

  void reset_simulation();

private:
  Node::SharedPtr nodeHandle_;
};
} // namespace clear