#include "generation/TrajectorGeneration.h"

#include <core/misc/Benchmark.h>
#include <core/misc/NumericTraits.h>
#include <pinocchio/Orientation.h>
#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace clear {

TrajectorGeneration::TrajectorGeneration(Node::SharedPtr nodeHandle)
    : nodeHandle_(nodeHandle) {
  const std::string config_file_ = nodeHandle_->get_parameter("/config_file")
                                       .get_parameter_value()
                                       .get<std::string>();

  auto config_ = YAML::LoadFile(config_file_);

  std::string model_package = config_["model"]["package"].as<std::string>();
  std::string urdf =
      ament_index_cpp::get_package_share_directory(model_package) +
      config_["model"]["urdf"].as<std::string>();
  RCLCPP_INFO(rclcpp::get_logger("TrajectorGeneration"), "model file: %s",
              urdf.c_str());
  pinocchioInterface_ptr_ = std::make_shared<PinocchioInterface>(urdf.c_str());

  foot_names = config_["model"]["foot_names"].as<std::vector<std::string>>();

  base_name = config_["model"]["base_name"].as<std::string>();
  RCLCPP_INFO(rclcpp::get_logger("TrajectorGeneration"),
              "[TrajectorGeneration]  base name: %s", base_name.c_str());

  freq_ = config_["generation"]["frequency"].as<scalar_t>();
  RCLCPP_INFO(rclcpp::get_logger("TrajectorGeneration"),
              "[TrajectorGeneration] frequency: %f", freq_);

  contact_flag_ = {true, true};

  referenceBuffer_ = std::make_shared<ReferenceBuffer>();

  footholdOpt_ptr = std::make_shared<FootholdOptimization>(
      nodeHandle_, pinocchioInterface_ptr_, referenceBuffer_);

  baseOpt_ptr = std::make_shared<ConvexMPC>(
      nodeHandle_, pinocchioInterface_ptr_, referenceBuffer_);

  run_.push(true);

  inner_loop_thread_ = std::thread(&TrajectorGeneration::innerLoop, this);
}

TrajectorGeneration::~TrajectorGeneration() {
  run_.push(false);
  inner_loop_thread_.join();
}

void TrajectorGeneration::setHeightCmd(scalar_t h) {
  baseOpt_ptr->setHeightCmd(h);
}

void TrajectorGeneration::updateCurrentState(
    std::shared_ptr<vector_t> qpos_ptr, std::shared_ptr<vector_t> qvel_ptr) {
  qpos_ptr_buffer.push(qpos_ptr);
  qvel_ptr_buffer.push(qvel_ptr);
}

void TrajectorGeneration::updateModeSchedule(
    std::shared_ptr<ModeSchedule> mode_schedule) {
  referenceBuffer_->setModeSchedule(mode_schedule);
}

std::shared_ptr<ReferenceBuffer> TrajectorGeneration::getReferenceBuffer() {
  return referenceBuffer_;
}

void TrajectorGeneration::innerLoop() {
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  benchmark::RepeatedTimer timer_;
  rclcpp::Rate loop_rate(freq_);

  RCLCPP_INFO(rclcpp::get_logger("TrajectorGeneration"),
              "start generation loop");

  while (rclcpp::ok() && run_.get()) {
    timer_.startTimer();
    if (qpos_ptr_buffer.get().get() == nullptr ||
        qvel_ptr_buffer.get().get() == nullptr ||
        referenceBuffer_->getModeSchedule() == nullptr) {
      continue;
    } else {
      std::shared_ptr<vector_t> qpos_ptr = qpos_ptr_buffer.get();
      std::shared_ptr<vector_t> qvel_ptr = qvel_ptr_buffer.get();
      pinocchioInterface_ptr_->updateRobotState(*qpos_ptr, *qvel_ptr);

      baseOpt_ptr->generateTrajRef();

      footholdOpt_ptr->optimize();

      generateFootTraj();

      baseOpt_ptr->optimize();
    }
    timer_.endTimer();
    loop_rate.sleep();
  }
  RCLCPP_INFO(rclcpp::get_logger("TrajectorGeneration"),
              "[TrajectorGeneration] max time %f ms,  average time %f ms",
              timer_.getMaxIntervalInMilliseconds(),
              timer_.getAverageInMilliseconds());
}

void TrajectorGeneration::generateFootTraj() {
  auto footholds = referenceBuffer_->getFootholds();
  const scalar_t t_now = nodeHandle_->now().seconds();
  auto mode_schedule = referenceBuffer_->getModeSchedule();
  if (xf_start_.empty()) {
    for (size_t k = 0; k < foot_names.size(); k++) {
      const auto &foot_name = foot_names[k];
      vector3_t pos =
          pinocchioInterface_ptr_->getFramePose(foot_name).translation();
      std::pair<scalar_t, vector3_t> xs;
      xs.first = nodeHandle_->now().seconds();
      xs.second = pos;
      xf_start_[foot_name] = std::move(xs);
    }
  }
  if (xf_end_.empty()) {
    for (size_t k = 0; k < foot_names.size(); k++) {
      const auto &foot_name = foot_names[k];
      vector3_t pos =
          pinocchioInterface_ptr_->getFramePose(foot_name).translation();
      std::pair<scalar_t, vector3_t> xe;
      xe.first = t_now + mode_schedule.get()->duration();
      xe.second = pos;
      xf_end_[foot_name] = std::move(xe);
    }
  }
  std::map<std::string, std::shared_ptr<CubicSplineTrajectory>> foot_pos_traj;

  auto contact_flag =
      biped::modeNumber2StanceLeg(mode_schedule->getModeFromPhase(0.0));
  for (size_t k = 0; k < foot_names.size(); k++) {
    const auto &foot_name = foot_names[k];
    vector3_t pos =
        pinocchioInterface_ptr_->getFramePose(foot_name).translation();
    if (contact_flag_[k] != contact_flag[k]) {
      std::pair<scalar_t, vector3_t> xs;
      xs.first = t_now - numeric_traits::limitEpsilon<scalar_t>();
      xs.second = pos;
      xf_start_[foot_name] = std::move(xs);
    }
    if (contact_flag[k]) {
      if (contact_flag_[k] != contact_flag[k]) {
        std::pair<scalar_t, vector3_t> xe;
        xe.first = t_now + mode_schedule->timeLeftInMode(0.0);
        xe.second = pos;
        xf_end_[foot_name] = std::move(xe);
      }
    } else {
      std::pair<scalar_t, vector3_t> xe;
      xe.first = footholds[foot_name].first;
      xe.second = footholds[foot_name].second;
      xf_end_[foot_name] = std::move(xe);
    }
    if (xf_start_[foot_name].first <
        xf_end_[foot_name].first - mode_schedule.get()->duration()) {
      xf_start_[foot_name].first =
          xf_end_[foot_name].first - mode_schedule.get()->duration();
      xf_start_[foot_name].second = pos;
    }

    vector_t base_pos =
        pinocchioInterface_ptr_->getFramePose(base_name).translation();
    xf_start_[foot_name].second.z() =
        std::min(xf_start_[foot_name].second.z(), base_pos.z() - 0.2);
    xf_end_[foot_name].second.z() =
        std::min(xf_end_[foot_name].second.z(), base_pos.z() - 0.2);

    std::vector<scalar_t> time;
    std::vector<vector_t> pos_t;
    time.push_back(xf_start_[foot_name].first);
    pos_t.push_back(xf_start_[foot_name].second);
    time.push_back(0.5 *
                   (xf_start_[foot_name].first + xf_end_[foot_name].first));
    vector3_t middle_pos =
        0.5 * (xf_start_[foot_name].second + xf_end_[foot_name].second);
    middle_pos.z() += contact_flag[k] ? 0.0 : 0.1;
    pos_t.push_back(middle_pos);
    time.push_back(xf_end_[foot_name].first);
    pos_t.push_back(xf_end_[foot_name].second);

    /* std::cout << "############# " << foot_name << ": " << pos.transpose()
              << " ##############\n";
    for (size_t i = 0; i < time.size(); i++) {
      std::cout << " t: " << time[i] - time.front()
                << " pos: " << pos_t[i].transpose() << "\n";
    } */

    auto cubicspline_ = std::make_shared<CubicSplineTrajectory>(
        3, CubicSplineInterpolation::SplineType::cspline);
    cubicspline_->set_boundary(
        CubicSplineInterpolation::BoundaryType::first_deriv, vector3_t::Zero(),
        CubicSplineInterpolation::BoundaryType::first_deriv, vector3_t::Zero());
    cubicspline_->fit(time, pos_t);
    /* std::cout << "############# opt " << foot_name << " ##############\n";
    for (size_t i = 0; i < time.size(); i++) {
      std::cout
          << " t: " << time[i] - time.front() << " pos: "
          << foot_traj_ptr->pos_trajectoryPtr->evaluate(time[i]).transpose()
          << "\n";
    } */
    foot_pos_traj[foot_name] = std::move(cubicspline_);
  }
  contact_flag_ = contact_flag;
  referenceBuffer_->setFootPosTraj(foot_pos_traj);
}

void TrajectorGeneration::setVelCmd(vector3_t vd, scalar_t yawd) {
  baseOpt_ptr->setVelCmd(vd, yawd);
}

}  // namespace clear
