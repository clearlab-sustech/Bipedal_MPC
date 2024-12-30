#pragma once
#include "control/WholeBodyController.h"
#include <core/gait/ModeSchedule.h>
#include <core/gait/MotionPhaseDefinition.h>
#include <core/misc/Buffer.h>
#include <core/trajectory/ReferenceBuffer.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <trans/msg/actuator_cmds.hpp>

using namespace rclcpp;

namespace clear {
class TrajectoryStabilization {

public:
  TrajectoryStabilization(Node::SharedPtr nodeHandle);

  ~TrajectoryStabilization();

  void updateCurrentState(std::shared_ptr<vector_t> qpos_ptr,
                          std::shared_ptr<vector_t> qvel_ptr);

  void updateReferenceBuffer(std::shared_ptr<ReferenceBuffer> referenceBuffer);

  trans::msg::ActuatorCmds::SharedPtr getCmds();

private:
  void publishCmds();

  void innerLoop();

private:
  Node::SharedPtr nodeHandle_;
  rclcpp::Publisher<trans::msg::ActuatorCmds>::SharedPtr
      actuators_cmds_pub_ptr_;

  std::shared_ptr<PinocchioInterface> pinocchioInterface_ptr_;
  std::shared_ptr<WholeBodyController> wbcPtr_;

  Buffer<std::shared_ptr<vector_t>> qpos_ptr_buffer, qvel_ptr_buffer;
  std::shared_ptr<ReferenceBuffer> referenceBuffer_;

  std::thread inner_loop_thread_;
  Buffer<bool> run_;
  scalar_t freq_;
  bool use_vector_field;

  std::string base_name, robot_name;
  std::vector<std::string> actuated_joints_name;
  std::vector<scalar_t> joints_default_pos;
  Buffer<std::shared_ptr<ActuatorCommands>> actuator_commands_buffer;
  std::string log_dir;
};

} // namespace clear
