#include "control/TrajectoryStabilization.h"

#include <core/misc/Benchmark.h>
#include <pinocchio/Orientation.h>
#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>

namespace clear {

TrajectoryStabilization::TrajectoryStabilization(Node::SharedPtr nodeHandle)
    : nodeHandle_(nodeHandle) {
  const std::string config_file_ = nodeHandle_->get_parameter("/config_file")
                                       .get_parameter_value()
                                       .get<std::string>();

  auto config_ = YAML::LoadFile(config_file_);
  std::string topic_prefix =
      config_["global"]["topic_prefix"].as<std::string>();
  std::string actuators_cmds_topic =
      config_["global"]["topic_names"]["actuators_cmds"].as<std::string>();
  log_dir = config_["global"]["log_dir"].as<std::string>();
  freq_ = config_["controller"]["frequency"].as<scalar_t>();
  RCLCPP_INFO(rclcpp::get_logger("TrajectoryStabilization"), "frequency: %f",
              freq_);

  actuated_joints_name =
      config_["model"]["actuated_joints_name"].as<std::vector<std::string>>();
  joints_default_pos =
      config_["model"]["default"]["joint_pos"].as<std::vector<scalar_t>>();
  robot_name = config_["model"]["name"].as<std::string>();

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_default);
  actuators_cmds_pub_ptr_ =
      nodeHandle_->create_publisher<trans::msg::ActuatorCmds>(
          topic_prefix + actuators_cmds_topic, qos);

  std::string model_package = config_["model"]["package"].as<std::string>();
  std::string urdf =
      ament_index_cpp::get_package_share_directory(model_package) +
      config_["model"]["urdf"].as<std::string>();
  RCLCPP_INFO(rclcpp::get_logger("TrajectoryStabilization"), "model file: %s",
              urdf.c_str());
  pinocchioInterface_ptr_ = std::make_shared<PinocchioInterface>(urdf.c_str());

  base_name = config_["model"]["base_name"].as<std::string>();

  run_.push(true);

  wbcPtr_ = std::make_shared<WholeBodyController>(nodeHandle_);
  inner_loop_thread_ = std::thread(&TrajectoryStabilization::innerLoop, this);
}

TrajectoryStabilization::~TrajectoryStabilization() {
  run_.push(false);
  inner_loop_thread_.join();
}

void TrajectoryStabilization::updateCurrentState(
    std::shared_ptr<vector_t> qpos_ptr, std::shared_ptr<vector_t> qvel_ptr) {
  qpos_ptr_buffer.push(qpos_ptr);
  qvel_ptr_buffer.push(qvel_ptr);
}

void TrajectoryStabilization::updateReferenceBuffer(
    std::shared_ptr<ReferenceBuffer> referenceBuffer) {
  referenceBuffer_ = referenceBuffer;
}

trans::msg::ActuatorCmds::SharedPtr TrajectoryStabilization::getCmds() {
  if (actuator_commands_buffer.get() == nullptr) {
    return nullptr;
  }
  auto actuator_commands_ = actuator_commands_buffer.get();

  const auto &model = pinocchioInterface_ptr_->getModel();
  trans::msg::ActuatorCmds::SharedPtr msg =
      std::make_shared<trans::msg::ActuatorCmds>();
  msg->header.frame_id = robot_name;
  msg->header.stamp = nodeHandle_->now();
  for (const auto &joint_name : actuated_joints_name) {
    if (model.existJointName(joint_name)) {
      msg->names.emplace_back(joint_name);
      pin::Index id = model.getJointId(joint_name) - 2;
      msg->gain_p.emplace_back(actuator_commands_->Kp(id));
      msg->pos_des.emplace_back(actuator_commands_->pos(id));
      msg->gaid_d.emplace_back(actuator_commands_->Kd(id));
      msg->vel_des.emplace_back(actuator_commands_->vel(id));
      msg->feedforward_torque.emplace_back(actuator_commands_->torque(id));
    }
  }
  return msg;
}

void TrajectoryStabilization::publishCmds() {
  if (actuator_commands_buffer.get() == nullptr) {
    return;
  }
  auto actuator_commands_ = actuator_commands_buffer.get();

  const auto &model = pinocchioInterface_ptr_->getModel();

  trans::msg::ActuatorCmds msg;
  msg.header.frame_id = robot_name;
  msg.header.stamp = nodeHandle_->now();

  for (const auto &joint_name : actuated_joints_name) {
    if (model.existJointName(joint_name)) {
      msg.names.emplace_back(joint_name);
      pin::Index id = model.getJointId(joint_name) - 2;
      msg.gain_p.emplace_back(actuator_commands_->Kp(id));
      msg.pos_des.emplace_back(actuator_commands_->pos(id));
      msg.gaid_d.emplace_back(actuator_commands_->Kd(id));
      msg.vel_des.emplace_back(actuator_commands_->vel(id));
      msg.feedforward_torque.emplace_back(actuator_commands_->torque(id));
    }
  }
  actuators_cmds_pub_ptr_->publish(msg);
}

void TrajectoryStabilization::innerLoop() {
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  benchmark::RepeatedTimer timer_;
  rclcpp::Rate loop_rate(freq_);

  scalar_t percentage = 1.2;
  vector_t qpos_start;

  while (rclcpp::ok() && run_.get()) {
    timer_.startTimer();
    if (qpos_ptr_buffer.get() == nullptr || qvel_ptr_buffer.get() == nullptr ||
        referenceBuffer_ == nullptr) {
      std::this_thread::sleep_for(
          std::chrono::milliseconds(int64_t(1000 / freq_)));
    } else if (percentage < 1.0) {
      if (percentage < 0.0) {
        qpos_start = qpos_ptr_buffer.get()->tail(actuated_joints_name.size());
        percentage = 0.0;
      }
      auto actuator_commands_ = std::make_shared<ActuatorCommands>();
      actuator_commands_->setZero(actuated_joints_name.size());

      const auto &model = pinocchioInterface_ptr_->getModel();
      for (size_t i = 0; i < actuated_joints_name.size(); i++) {
        auto joint_name = actuated_joints_name[i];
        if (model.existJointName(joint_name)) {
          pin::Index id = model.getJointId(joint_name) - 2;
          actuator_commands_->Kp[id] = 100.0;
          actuator_commands_->Kd[id] = 3.0;
          actuator_commands_->pos[id] = qpos_start[id] * (1.0 - percentage) +
                                        percentage * joints_default_pos[i];
          actuator_commands_->vel[id] = 0.0;
          actuator_commands_->torque[id] = 0.0;
        }
      }
      percentage += 1.0 / (3.0 * freq_);
      actuator_commands_buffer.push(actuator_commands_);
      publishCmds();
      if (percentage > 1.0 - 1.0 / (3.0 * freq_)) {
        RCLCPP_INFO(rclcpp::get_logger("TrajectoryStabilization"),
                    "switch to WBC");
      }
    } else if (referenceBuffer_->isReady()) {
      std::shared_ptr<vector_t> qpos_ptr = qpos_ptr_buffer.get();
      std::shared_ptr<vector_t> qvel_ptr = qvel_ptr_buffer.get();
      pinocchioInterface_ptr_->updateRobotState(*qpos_ptr, *qvel_ptr);

      wbcPtr_->updateState(qpos_ptr, qvel_ptr);
      wbcPtr_->updateReferenceBuffer(referenceBuffer_);
      actuator_commands_buffer.push(wbcPtr_->optimize());
      publishCmds();
    }
    timer_.endTimer();
    loop_rate.sleep();
  }
  RCLCPP_INFO(rclcpp::get_logger("TrajectoryStabilization"),
              "max time %f ms,  average time %f ms",
              timer_.getMaxIntervalInMilliseconds(),
              timer_.getAverageInMilliseconds());
}

}  // namespace clear
