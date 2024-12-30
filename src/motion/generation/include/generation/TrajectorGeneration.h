#pragma once

#include "generation/ConvexMPC.h"
#include "generation/FootholdOptimization.h"
#include <core/gait/ModeSchedule.h>
#include <core/gait/MotionPhaseDefinition.h>
#include <core/misc/Buffer.h>
#include <core/trajectory/ReferenceBuffer.h>
#include <memory>
#include <pinocchio/PinocchioInterface.h>
#include <rclcpp/rclcpp.hpp>
#include <string>

using namespace rclcpp;
using namespace std;

namespace clear {
class TrajectorGeneration {

public:
  TrajectorGeneration(Node::SharedPtr nodeHandle);

  ~TrajectorGeneration();

  void updateCurrentState(std::shared_ptr<vector_t> qpos_ptr,
                          std::shared_ptr<vector_t> qvel_ptr);

  void updateModeSchedule(std::shared_ptr<ModeSchedule> mode_schedule);

  std::shared_ptr<ReferenceBuffer> getReferenceBuffer();

  void setVelCmd(vector3_t vd, scalar_t yawd);

  void setHeightCmd(scalar_t h);

private:
  void innerLoop();

  void generateFootTraj();

private:
  Node::SharedPtr nodeHandle_;
  std::shared_ptr<PinocchioInterface> pinocchioInterface_ptr_;
  std::shared_ptr<ReferenceBuffer> referenceBuffer_;

  std::shared_ptr<FootholdOptimization> footholdOpt_ptr;
  std::shared_ptr<ConvexMPC> baseOpt_ptr;

  Buffer<std::shared_ptr<vector_t>> qpos_ptr_buffer;
  Buffer<std::shared_ptr<vector_t>> qvel_ptr_buffer;

  std::thread inner_loop_thread_, inner_lip_loop_thread_;
  Buffer<bool> run_;
  scalar_t freq_;
  std::string base_name;
  std::vector<std::string> foot_names;

  std::map<std::string, std::pair<scalar_t, vector3_t>> xf_start_;
  std::map<std::string, std::pair<scalar_t, vector3_t>> xf_end_;
  biped::contact_flag_t contact_flag_;
};

} // namespace clear
