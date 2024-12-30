#include <MotionManager.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <core/misc/Benchmark.h>
#include <pinocchio/Orientation.h>
#include <yaml-cpp/yaml.h>

namespace clear {

MotionManager::MotionManager() : Node("MotionManager") {
  this->declare_parameter("/config_file", "");
}

MotionManager::~MotionManager() {
  run_.push(false);
  inner_loop_thread_.join();
}

void MotionManager::init() {
  intializationPtr_ =
      std::make_shared<Initialization>(this->shared_from_this());

  joyStickPtr_ = std::make_shared<JoyStick>(this->shared_from_this());

  // while (!joyStickPtr_->isStart()) {
  //   rclcpp::spin_some(this->shared_from_this());
  // }

  estimatorPtr_ =
      std::make_shared<StateEstimationLKF>(this->shared_from_this());

  RCLCPP_INFO(this->get_logger(), "StateEstimationLKF Ready");

  gaitSchedulePtr_ = std::make_shared<GaitSchedule>(this->shared_from_this());

  RCLCPP_INFO(this->get_logger(), "GaitSchedule Ready");

  trajGenPtr_ = std::make_shared<TrajectorGeneration>(this->shared_from_this());

  RCLCPP_INFO(this->get_logger(), "TrajectorGeneration Ready");

  trajectoryStabilizationPtr_ =
      std::make_shared<TrajectoryStabilization>(this->shared_from_this());

  visPtr_ = std::make_shared<DataVisualization>(this->shared_from_this());

  const std::string config_file_ = this->get_parameter("/config_file")
                                       .get_parameter_value()
                                       .get<std::string>();

  auto config_ = YAML::LoadFile(config_file_);
  inner_loop_thread_ = std::thread(&MotionManager::innerLoop, this);
  run_.push(true);
}

void MotionManager::innerLoop() {
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  rclcpp::Rate loop_rate(1000.0);
  const scalar_t ts = this->now().seconds();

  while (rclcpp::ok() && run_.get()) {

    if (joyStickPtr_->eStop()) {
      RCLCPP_INFO(this->get_logger(), "e-stop");
      trajGenPtr_.reset();
      trajectoryStabilizationPtr_.reset();
      visPtr_.reset();
      break;
    }

    if (gaitSchedulePtr_->getCurrentGaitName() == "walk") {
      trajGenPtr_->setVelCmd(joyStickPtr_->getLinearVelCmd(),
                             joyStickPtr_->getYawVelCmd());
    }

    // if (this->now().seconds() > ts + 6.0 &&
    //     gaitSchedulePtr_->getCurrentGaitName() != "trot") {
    //   gaitSchedulePtr_->switchGait("trot");
    // }
    // if (this->now().seconds() > ts + 4.0 && this->now().seconds() < ts + 8.0) {
    //   trajGenPtr_->setVelCmd(vector3_t(0.0, 0.4 * (this->now().seconds() - ts - 4.0) / 4.0, 0.0), 0.0);
    // } else if (this->now().seconds() > ts + 8.0) {
    //   trajGenPtr_->setVelCmd(vector3_t(0.0, 0.4, 0.0), 0.0);
    // }
    // if (this->now().seconds() > ts + 2 * M_PI) {
    //   scalar_t t = this->now().seconds() - ts - 2 * M_PI;
    //   trajGenPtr_->setVelCmd(vector3_t(0.4 * sin(2 * t), 0.2 * sin(t), 0.0),
    //                          0.3 * sin(t));
    // }

    // trajGenPtr_->setHeightCmd(joyStickPtr_->getHeightCmd());

    scalar_t horizon_time_ =
        min(2.0, max(0.3, gaitSchedulePtr_->currentGaitCycle()));

    auto mode_schedule_ptr = gaitSchedulePtr_->eval(horizon_time_);

    estimatorPtr_->updateModeSchedule(mode_schedule_ptr);

    trajGenPtr_->updateCurrentState(estimatorPtr_->getQpos(),
                                    estimatorPtr_->getQvel());

    trajGenPtr_->updateModeSchedule(mode_schedule_ptr);

    trajectoryStabilizationPtr_->updateCurrentState(estimatorPtr_->getQpos(),
                                                    estimatorPtr_->getQvel());

    trajectoryStabilizationPtr_->updateReferenceBuffer(
        trajGenPtr_->getReferenceBuffer());

    visPtr_->updateCurrentState(estimatorPtr_->getQpos(),
                                estimatorPtr_->getQvel());
    visPtr_->updateReferenceBuffer(trajGenPtr_->getReferenceBuffer());

    loop_rate.sleep();
  }

  exit(0);
}

} // namespace clear
