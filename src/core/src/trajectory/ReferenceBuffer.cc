#include "core/trajectory/ReferenceBuffer.h"
#include <iostream>

namespace clear {

ReferenceBuffer::ReferenceBuffer() {}

ReferenceBuffer::~ReferenceBuffer() {}

void ReferenceBuffer::clearAll() {
  integ_base_rpy_buffer_.clear();
  lip_base_pos_buffer_.clear();
  lip_base_vel_buffer_.clear();
  optimized_base_pos_buffer_.clear();
  optimized_base_rpy_buffer_.clear();
  optimized_base_vel_buffer_.clear();
  optimized_base_omega_buffer_.clear();
  foot_rpy_buffer_.clear();
  foot_pos_buffer_.clear();
  lip_foot_pos_buffer_.clear();
  joints_pos_buffer_.clear();
}

bool ReferenceBuffer::isReady() { 
  bool is_ready_ = true;
  is_ready_ &= (integ_base_rpy_buffer_.get()!=nullptr);
  // is_ready_ &= (lip_base_pos_buffer_.get()!=nullptr);
  // is_ready_ &= (lip_base_vel_buffer_.get()!=nullptr);
  is_ready_ &= (optimized_base_pos_buffer_.get()!=nullptr);
  is_ready_ &= (optimized_base_rpy_buffer_.get()!=nullptr);
  is_ready_ &= (optimized_base_vel_buffer_.get()!=nullptr);
  is_ready_ &= (optimized_base_omega_buffer_.get()!=nullptr);
  is_ready_ &= (mode_schedule_buffer_.get()!=nullptr);
  is_ready_ &= !foot_pos_buffer_.get().empty();
  // is_ready_ &= !lip_foot_pos_buffer_.get().empty();
  is_ready_ &= !footholds_buffer_.get().empty();
  return is_ready_; 
}

std::shared_ptr<CubicSplineTrajectory>
ReferenceBuffer::getIntegratedBaseRpyTraj() const {
  return integ_base_rpy_buffer_.get();
}

std::shared_ptr<CubicSplineTrajectory>
ReferenceBuffer::getIntegratedBasePosTraj() const {
  return integ_base_pos_buffer_.get();
}

std::shared_ptr<CubicSplineTrajectory>
ReferenceBuffer::getLipBasePosTraj() const {
  return lip_base_pos_buffer_.get();
}

std::shared_ptr<CubicSplineTrajectory>
ReferenceBuffer::getLipBaseVelTraj() const {
  return lip_base_vel_buffer_.get();
}

std::shared_ptr<CubicSplineTrajectory>
ReferenceBuffer::getOptimizedBaseRpyTraj() const {
  if (optimized_base_rpy_buffer_.get() != nullptr) {
    return optimized_base_rpy_buffer_.get();
  } else {
    return integ_base_rpy_buffer_.get();
  }
}

std::shared_ptr<CubicSplineTrajectory>
ReferenceBuffer::getOptimizedBasePosTraj() const {
  if (optimized_base_pos_buffer_.get() != nullptr) {
    return optimized_base_pos_buffer_.get();
  } else {
    return lip_base_vel_buffer_.get();
  }
}

std::shared_ptr<CubicSplineTrajectory>
ReferenceBuffer::getOptimizedBaseVelTraj() const {
  return optimized_base_vel_buffer_.get();
}

std::shared_ptr<CubicSplineTrajectory>
ReferenceBuffer::getOptimizedBaseOmegaTraj() const {
  return optimized_base_omega_buffer_.get();
}

std::shared_ptr<CubicSplineTrajectory> ReferenceBuffer::getOptimizedForceTraj() const
{
  return optimized_force_buffer_.get();
}


std::map<std::string, std::shared_ptr<CubicSplineTrajectory>>
ReferenceBuffer::getFootRpyTraj() const {
  return foot_rpy_buffer_.get();
}

std::map<std::string, std::shared_ptr<CubicSplineTrajectory>>
ReferenceBuffer::getFootPosTraj() const {
  return foot_pos_buffer_.get();
}

std::map<std::string, std::shared_ptr<CubicSplineTrajectory>>
ReferenceBuffer::getLipFootPosTraj() const {
  return lip_foot_pos_buffer_.get();
}

std::shared_ptr<CubicSplineTrajectory>
ReferenceBuffer::getJointsPosTraj() const {
  return joints_pos_buffer_.get();
}

std::map<std::string, std::pair<scalar_t, vector3_t>>
ReferenceBuffer::getFootholds() const {
  return footholds_buffer_.get();
}

std::shared_ptr<ModeSchedule> ReferenceBuffer::getModeSchedule() const {
  return mode_schedule_buffer_.get();
}

void ReferenceBuffer::setIntegratedBaseRpyTraj(
    std::shared_ptr<CubicSplineTrajectory> base_rpy_traj) {
  integ_base_rpy_buffer_.push(base_rpy_traj);
}

void ReferenceBuffer::setIntegratedBasePosTraj(
    std::shared_ptr<CubicSplineTrajectory> base_pos_traj) {
  integ_base_pos_buffer_.push(base_pos_traj);
}

void ReferenceBuffer::setLipBasePosTraj(
    std::shared_ptr<CubicSplineTrajectory> base_pos_traj) {
  lip_base_pos_buffer_.push(base_pos_traj);
}

void ReferenceBuffer::setLipBaseVelTraj(
    std::shared_ptr<CubicSplineTrajectory> base_vel_traj) {
  lip_base_vel_buffer_.push(base_vel_traj);
}

void ReferenceBuffer::setOptimizedBasePosTraj(
    std::shared_ptr<CubicSplineTrajectory> base_pos_traj) {
  optimized_base_pos_buffer_.push(base_pos_traj);
}

void ReferenceBuffer::setOptimizedBaseRpyTraj(
    std::shared_ptr<CubicSplineTrajectory> base_rpy_traj) {
  optimized_base_rpy_buffer_.push(base_rpy_traj);
}

void ReferenceBuffer::setOptimizedBaseVelTraj(
    std::shared_ptr<CubicSplineTrajectory> base_vel_traj) {
  optimized_base_vel_buffer_.push(base_vel_traj);
}

void ReferenceBuffer::setOptimizedBaseOmegaTraj(
    std::shared_ptr<CubicSplineTrajectory> base_omega_traj) {
  optimized_base_omega_buffer_.push(base_omega_traj);
}

void ReferenceBuffer::setOptimizedForceTraj(
    std::shared_ptr<CubicSplineTrajectory> force_traj)
{
  optimized_force_buffer_.push(force_traj);
}

void ReferenceBuffer::setFootRpyTraj(
    std::map<std::string, std::shared_ptr<CubicSplineTrajectory>>
        foot_rpy_traj) {
  foot_rpy_buffer_.push(foot_rpy_traj);
}

void ReferenceBuffer::setFootPosTraj(
    std::map<std::string, std::shared_ptr<CubicSplineTrajectory>>
        foot_pos_traj) {
  foot_pos_buffer_.push(foot_pos_traj);
}

void ReferenceBuffer::setLipFootPosTraj(
    std::map<std::string, std::shared_ptr<CubicSplineTrajectory>>
        foot_pos_traj) {
  lip_foot_pos_buffer_.push(foot_pos_traj);
}

void ReferenceBuffer::setJointsPosTraj(
    std::shared_ptr<CubicSplineTrajectory> joints_pos_traj) {
  joints_pos_buffer_.push(joints_pos_traj);
}

void ReferenceBuffer::setFootholds(
    std::map<std::string, std::pair<scalar_t, vector3_t>> footholds) {
  footholds_buffer_.push(footholds);
}

void ReferenceBuffer::setModeSchedule(
    std::shared_ptr<ModeSchedule> mode_schedule) {
  mode_schedule_buffer_.push(mode_schedule);
}

} // namespace clear