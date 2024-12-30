#include "estimation/StateEstimationLKF.h"

#include <core/gait/LegLogic.h>
#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace clear {

StateEstimationLKF::StateEstimationLKF(Node::SharedPtr nodeHandle)
    : nodeHandle_(nodeHandle) {
  const std::string config_file_ = nodeHandle_->get_parameter("/config_file")
                                       .get_parameter_value()
                                       .get<std::string>();

  auto config_ = YAML::LoadFile(config_file_);

  robot_name = config_["model"]["name"].as<std::string>();
  RCLCPP_INFO(rclcpp::get_logger("StateEstimationLKF"), "robot_name: %s",
              robot_name.c_str());

  std::string model_package = config_["model"]["package"].as<std::string>();
  std::string urdf =
      ament_index_cpp::get_package_share_directory(model_package) +
      config_["model"]["urdf"].as<std::string>();
  RCLCPP_INFO(rclcpp::get_logger("StateEstimationLKF"), "model file: %s",
              urdf.c_str());

  pinocchioInterface_ptr = std::make_unique<PinocchioInterface>(urdf.c_str());
  foot_names = config_["model"]["foot_names"].as<std::vector<std::string>>();
  dt_ = config_["estimation"]["dt"].as<scalar_t>();
  use_odom_ = config_["estimation"]["use_odom"].as<bool>();

  RCLCPP_INFO(rclcpp::get_logger("StateEstimationLKF"), "dt: %f", dt_);
  RCLCPP_INFO(rclcpp::get_logger("StateEstimationLKF"), "use odom: %s",
              use_odom_ ? "true" : "false");
  for (const auto &name : foot_names) {
    RCLCPP_INFO(rclcpp::get_logger("StateEstimationLKF"), "foot name: %s",
                name.c_str());
    cflag_.emplace_back(true);
  }

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
  std::string topic_prefix =
      config_["global"]["topic_prefix"].as<std::string>();
  std::string imu_topic =
      config_["global"]["topic_names"]["imu"].as<std::string>();
  imu_subscription_ = nodeHandle_->create_subscription<sensor_msgs::msg::Imu>(
      topic_prefix + imu_topic, qos,
      std::bind(&StateEstimationLKF::imuCallback, this, std::placeholders::_1));

  std::string joints_topic =
      config_["global"]["topic_names"]["joints_state"].as<std::string>();
  joints_state_subscription_ =
      nodeHandle_->create_subscription<sensor_msgs::msg::JointState>(
          topic_prefix + joints_topic, qos,
          std::bind(&StateEstimationLKF::jointCallback, this,
                    std::placeholders::_1));

  std::string odom_topic =
      config_["global"]["topic_names"]["odom"].as<std::string>();
  odom_subscription_ =
      nodeHandle_->create_subscription<nav_msgs::msg::Odometry>(
          topic_prefix + odom_topic, qos,
          std::bind(&StateEstimationLKF::odomCallback, this,
                    std::placeholders::_1));
  odom_est_publisher_ = nodeHandle_->create_publisher<nav_msgs::msg::Odometry>(
      topic_prefix + odom_topic + "_est", qos);

  inner_loop_thread_ = std::thread(&StateEstimationLKF::innerLoop, this);
  run_.push(true);
  setup();
}

StateEstimationLKF::~StateEstimationLKF() {
  run_.push(false);
  inner_loop_thread_.join();
}

void StateEstimationLKF::setup() {
  x_est.setZero();
  x_est.z() = 0.5;
  ps_.setZero();
  vs_.setZero();

  A_.setIdentity();
  A_.block<3, 3>(0, 3).diagonal().fill(dt_);

  B_.setZero();
  B_.block<3, 3>(3, 0).diagonal().fill(dt_);

  matrix_t C1 = matrix_t::Zero(3, 6);
  C1.leftCols(3).setIdentity();
  matrix_t C2 = matrix_t::Zero(3, 6);
  C2.rightCols(3).setIdentity();

  C_.setZero();
  C_.block<3, 6>(0, 0) = C1;
  C_.block<3, 6>(3, 0) = C1;
  C_.block<6, 6>(0, 6).diagonal().fill(-1.0);
  C_.block<3, 6>(6, 0) = C2;
  C_.block<3, 6>(9, 0) = C2;
  C_(12, 8) = 1.0;
  C_(13, 11) = 1.0;

  Sigma_.setZero();
  Sigma_.diagonal().fill(100.0);
  Q0_.setIdentity();
  Q0_.topLeftCorner(3, 3).diagonal().fill(dt_ / 20.0);
  Q0_.block<3, 3>(3, 3).diagonal().fill(dt_ * 9.81 / 20.0);
  Q0_.bottomRightCorner(6, 6).diagonal().fill(dt_);
  R0_.setIdentity();
}

void StateEstimationLKF::setContactFlag(vector<bool> flag) { cflag_ = flag; }

void StateEstimationLKF::updateModeSchedule(
    std::shared_ptr<ModeSchedule> mode_schedule) {
  mode_schedule_buffer_.push(mode_schedule);
}

void StateEstimationLKF::angularMotionEstimate(
    const sensor_msgs::msg::Imu &imu_data, std::shared_ptr<vector_t> qpos,
    std::shared_ptr<vector_t> qvel) {
  const auto &quat_imu = imu_data.orientation;
  Eigen::Quaternion<scalar_t> quat(quat_imu.w, quat_imu.x, quat_imu.y,
                                   quat_imu.z);
  qpos->segment(3, 4) << quat_imu.x, quat_imu.y, quat_imu.z, quat_imu.w;
  qvel->segment(3, 3) << imu_data.angular_velocity.x,
      imu_data.angular_velocity.y, imu_data.angular_velocity.z;
}

void StateEstimationLKF::linearMotionEstimate(
    const sensor_msgs::msg::Imu &imu_data, std::shared_ptr<vector_t> qpos,
    std::shared_ptr<vector_t> qvel) {
  Eigen::Quaternion<scalar_t> quat(
      imu_data.orientation.w, imu_data.orientation.x, imu_data.orientation.y,
      imu_data.orientation.z);
  vector3_t b_acc(imu_data.linear_acceleration.x,
                  imu_data.linear_acceleration.y,
                  imu_data.linear_acceleration.z);
  vector3_t w_acc = quat.toRotationMatrix() * b_acc;
  vector3_t g(0, 0, -9.81);
  w_acc = w_acc + g;

  scalar_t noise_pimu = 0.02;
  scalar_t noise_vimu = 0.02;
  scalar_t noise_pfoot = 0.002;
  scalar_t noise_pimu_rel_foot = 0.001;
  scalar_t noise_vimu_rel_foot = 0.1;
  scalar_t noise_zfoot = 0.001;

  matrix_t Q = matrix_t::Identity(12, 12);
  Q.topLeftCorner(3, 3) = Q0_.topLeftCorner(3, 3) * noise_pimu;
  Q.block<3, 3>(3, 3) = Q0_.block<3, 3>(3, 3) * noise_vimu;
  Q.block<6, 6>(6, 6) = Q0_.block<6, 6>(6, 6) * noise_pfoot;

  matrix_t R = matrix_t::Identity(14, 14);
  R.topLeftCorner(6, 6) = R0_.topLeftCorner(6, 6) * noise_pimu_rel_foot;
  R.block<6, 6>(6, 6) = R0_.block<6, 6>(6, 6) * noise_vimu_rel_foot;
  R.block<2, 2>(12, 12) = R0_.block<2, 2>(12, 12) * noise_zfoot;

  size_t q_idx = 0;
  size_t idx1 = 0;
  size_t idx2 = 0;

  vector4_t pzs = vector4_t::Zero();
  vector3_t p0, v0;
  p0 << x_est[0], x_est[1], x_est[2];
  v0 << x_est[3], x_est[4], x_est[5];

  pinocchioInterface_ptr->updateRobotState(*qpos, *qvel);
  auto mode_schedule = mode_schedule_buffer_.get();
  auto contact_flag =
      biped::modeNumber2StanceLeg(mode_schedule->getModeFromPhase(0.0));
  auto ntlo = biped::getTimeOfNextLiftOff(0, mode_schedule);
  const scalar_t gait_cycle = mode_schedule->gaitCycle();

  bool release_ = (abs(w_acc.z()) < 1.0);
  for (auto &flag : contact_flag) {
    release_ &= (!flag);
  }

  for (size_t i = 0; i < foot_names.size(); i++) {
    auto pose = pinocchioInterface_ptr->getFramePose(foot_names[i]);
    auto v =
        pinocchioInterface_ptr->getFrame6dVel_localWorldAligned(foot_names[i]);
    vector3_t p_f = pose.translation();
    p_f.z() -= 0.02;  // the foot radius, be careful
    vector3_t v_f = v.linear();

    int i1 = 3 * i;
    q_idx = 6 + i1;
    idx1 = 6 + i1;
    idx2 = 12 + i;

    scalar_t trust = 1.0;
    if (contact_flag[i] || release_) {
      const scalar_t trust_window = 0.15;
      scalar_t contact_phase = 1.0 - ntlo[i] / (0.5 * gait_cycle);
      if (contact_phase < trust_window) {
        trust = contact_phase / trust_window;
      } else if (contact_phase > (1.0 - trust_window)) {
        trust = (1.0 - contact_phase) / trust_window;
      }
    } else {
      trust = 0.0;
    }

    scalar_t high_suspect_number = 500.0;

    Q.block<3, 3>(q_idx, q_idx) = (1.0 + (1.0 - trust) * high_suspect_number) *
                                  Q.block<3, 3>(q_idx, q_idx);
    R.block<3, 3>(idx1, idx1) =
        (1.0 + (1.0 - trust) * high_suspect_number) * R.block<3, 3>(idx1, idx1);
    R(idx2, idx2) = (1.0 + (1.0 - trust) * high_suspect_number) * R(idx2, idx2);
    ps_.segment(i1, 3) = -p_f;
    pzs(i) = (1.0 - trust) * (p0(2) + p_f(2));
    vs_.segment(i1, 3) = (1.0 - trust) * v0 + trust * (-v_f);
  }

  Eigen::Matrix<scalar_t, 14, 1> y;
  y << ps_, vs_, pzs;
  vector_t x_pred = A_ * x_est + B_ * w_acc;

  matrix_t Sigma_bar = A_ * Sigma_ * A_.transpose() + Q;
  matrix_t S = C_ * Sigma_bar * C_.transpose() + R;
  vector_t correct = S.lu().solve(y - C_ * x_pred);
  x_est = x_pred + Sigma_bar * C_.transpose() * correct;
  matrix_t SC_ = S.lu().solve(C_);
  Sigma_ = (matrix_t::Identity(12, 12) - Sigma_bar * C_.transpose() * SC_) *
           Sigma_bar;
  Sigma_ = 0.5 * (Sigma_ + Sigma_.transpose());
  if (Sigma_.topLeftCorner(2, 2).determinant() > 1e-6) {
    Sigma_.topRightCorner(2, 10).setZero();
    Sigma_.bottomLeftCorner(10, 2).setZero();
    Sigma_.topLeftCorner(2, 2) *= 0.1;
  }
  qpos->head(3) = x_est.head(3);
  qvel->head(3) = quat.toRotationMatrix().transpose() * x_est.segment(3, 3);
}

void StateEstimationLKF::innerLoop() {
  rclcpp::Rate loop_rate(1.0 / dt_);

  const scalar_t t0 = nodeHandle_->now().seconds();
  while (rclcpp::ok() && run_.get()) {
    if (imu_msg_buffer.get().get() == nullptr ||
        mode_schedule_buffer_.get().get() == nullptr ||
        joint_state_msg_buffer.get().get() == nullptr ||
        (use_odom_ && odom_msg_buffer.get().get() == nullptr)) {
      continue;
    }
    // do not delete these three lines
    sensor_msgs::msg::Imu::SharedPtr imu_msg = imu_msg_buffer.get();
    sensor_msgs::msg::JointState::SharedPtr joint_state_msg =
        joint_state_msg_buffer.get();
    nav_msgs::msg::Odometry::SharedPtr odom_msg = odom_msg_buffer.get();

    const pin::Model &model_ = pinocchioInterface_ptr->getModel();
    std::shared_ptr<vector_t> qpos_ptr_ = std::make_shared<vector_t>(model_.nq);
    qpos_ptr_->setZero();
    std::shared_ptr<vector_t> qvel_ptr_ = std::make_shared<vector_t>(model_.nv);
    qvel_ptr_->setZero();

    for (size_t k = 0; k < joint_state_msg->name.size(); k++) {
      if (static_cast<int>(k) < model_.njoints &&
          model_.existJointName(joint_state_msg->name[k])) {
        pin::Index id = model_.getJointId(joint_state_msg->name[k]) - 2;
        (*qpos_ptr_)[id + 7] = joint_state_msg->position[k];
        (*qvel_ptr_)[id + 6] = joint_state_msg->velocity[k];
      }
    }

    if (use_odom_) {
      const auto &orientation = odom_msg->pose.pose.orientation;
      const auto &position = odom_msg->pose.pose.position;
      const auto &vel = odom_msg->twist.twist.linear;
      const auto &ang_vel = odom_msg->twist.twist.angular;

      Eigen::Quaternion<scalar_t> quat(orientation.w, orientation.x,
                                       orientation.y, orientation.z);
      matrix3_t rot = quat.toRotationMatrix();
      (*qpos_ptr_).head(3) << position.x, position.y, position.z;
      (*qvel_ptr_).head(3) << vel.x, vel.y, vel.z;
      (*qvel_ptr_).head(3) = rot.transpose() * (*qvel_ptr_).head(3);

      (*qpos_ptr_).segment(3, 4) << orientation.x, orientation.y, orientation.z,
          orientation.w;
      (*qvel_ptr_).segment(3, 3) << ang_vel.x, ang_vel.y, ang_vel.z;
    } else {
      // const auto touch_sensor_data = touch_msg_buffer.get()->value;
      // for (auto &data : touch_sensor_data) {
      //   RCLCPP_INFO(rclcpp::get_logger("StateEstimationLKF"),
      //   "touch_sensor_data: %f", data);
      // }
      angularMotionEstimate(*imu_msg, qpos_ptr_, qvel_ptr_);
      linearMotionEstimate(*imu_msg, qpos_ptr_, qvel_ptr_);
    }

    if (nodeHandle_->now().seconds() - t0 > 0.3) {
      qpos_ptr_buffer.push(qpos_ptr_);
      qvel_ptr_buffer.push(qvel_ptr_);
    }
    auto message = nav_msgs::msg::Odometry();
    {
      message.header.frame_id = robot_name;
      message.header.stamp = nodeHandle_->now();
      message.pose.pose.position.x = qpos_ptr_->x();
      message.pose.pose.position.y = qpos_ptr_->y();
      message.pose.pose.position.z = qpos_ptr_->z();
      message.pose.pose.orientation.w = (*qpos_ptr_)[6];
      message.pose.pose.orientation.x = (*qpos_ptr_)[3];
      message.pose.pose.orientation.y = (*qpos_ptr_)[4];
      message.pose.pose.orientation.z = (*qpos_ptr_)[5];

      Eigen::Quaternion<scalar_t> quat((*qpos_ptr_)[6], (*qpos_ptr_)[3],
                                       (*qpos_ptr_)[4], (*qpos_ptr_)[5]);
      vector3_t vel_world = quat.toRotationMatrix() * qvel_ptr_->head(3);
      message.twist.twist.linear.x = vel_world.x();
      message.twist.twist.linear.y = vel_world.y();
      message.twist.twist.linear.z = vel_world.z();
      message.twist.twist.angular.x = (*qvel_ptr_)[3];
      message.twist.twist.angular.y = (*qvel_ptr_)[4];
      message.twist.twist.angular.z = (*qvel_ptr_)[5];
    }
    odom_est_publisher_->publish(message);

    /* RCLCPP_INFO_STREAM(rclcpp::get_logger("StateEstimationLKF"),
                       "qpos: " << (*qpos_ptr_).transpose() << "\n");
    RCLCPP_INFO_STREAM(rclcpp::get_logger("StateEstimationLKF"),
                       "qvel: " << (*qvel_ptr_).transpose() << "\n"); */

    loop_rate.sleep();
  }
}

std::shared_ptr<vector_t> StateEstimationLKF::getQpos() {
  return qpos_ptr_buffer.get();
}

std::shared_ptr<vector_t> StateEstimationLKF::getQvel() {
  return qvel_ptr_buffer.get();
}

void StateEstimationLKF::imuCallback(
    const sensor_msgs::msg::Imu::SharedPtr msg) const {
  if (msg->header.frame_id == robot_name) {
    imu_msg_buffer.push(msg);
  }
}

void StateEstimationLKF::jointCallback(
    const sensor_msgs::msg::JointState::SharedPtr msg) const {
  if (msg->header.frame_id == robot_name) {
    joint_state_msg_buffer.push(msg);
  }
}

void StateEstimationLKF::setImuMsg(sensor_msgs::msg::Imu::SharedPtr msg) {
  if (msg->header.frame_id == robot_name) {
    imu_msg_buffer.push(msg);
  }
}

void StateEstimationLKF::setJointsMsg(
    sensor_msgs::msg::JointState::SharedPtr msg) {
  if (msg->header.frame_id == robot_name) {
    joint_state_msg_buffer.push(msg);
  }
}

void StateEstimationLKF::odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) const {
  if (msg->header.frame_id == robot_name) {
    odom_msg_buffer.push(msg);
  }
}

}  // namespace clear