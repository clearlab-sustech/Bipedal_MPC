#include "sim/SimPublisher.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

namespace clear {

SimPublisher::SimPublisher(mj::Simulate *sim, const std::string config_yaml)
    : Node("SimPublisher"), sim_(sim) {

  auto config_ = YAML::LoadFile(config_yaml);
  std::string name_prefix = config_["global"]["topic_prefix"].as<std::string>();
  std::string model_package = config_["model"]["package"].as<std::string>();
  std::string model_file =
      ament_index_cpp::get_package_share_directory(model_package) +
      config_["model"]["xml"].as<std::string>();
  mju::strcpy_arr(sim_->filename, model_file.c_str());
  sim_->uiloadrequest.fetch_add(1);
  RCLCPP_INFO(rclcpp::get_logger("SimPublisher"), "model file: %s", model_file.c_str());

  std::string sim_reset_service =
      config_["global"]["service_names"]["sim_reset"].as<std::string>();
  reset_service_ = this->create_service<trans::srv::SimulationReset>(
      name_prefix + sim_reset_service,
      std::bind(&SimPublisher::reset_callback, this, std::placeholders::_1,
                std::placeholders::_2));

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_default);
  std::string imu_topic =
      config_["global"]["topic_names"]["imu"].as<std::string>();
  imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(
      name_prefix + imu_topic, qos);

  std::string joints_state_topic =
      config_["global"]["topic_names"]["joints_state"].as<std::string>();
  joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
      name_prefix + joints_state_topic, qos);

  std::string odom_topic =
      config_["global"]["topic_names"]["odom"].as<std::string>();
  odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
      name_prefix + odom_topic, qos);

  mjtNum freq_imu = config_["simulation"]["frequency"]["imu"].as<mjtNum>();
  timers_.emplace_back(this->create_wall_timer(
      std::chrono::duration<mjtNum, std::milli>{1000.0 / freq_imu},
      std::bind(&SimPublisher::imuCallback, this)));

  mjtNum freq_joints_state =
      config_["simulation"]["frequency"]["joints_state"].as<mjtNum>();
  timers_.emplace_back(this->create_wall_timer(
      std::chrono::duration<mjtNum, std::milli>{1000.0 / freq_joints_state},
      std::bind(&SimPublisher::jointCallback, this)));

  mjtNum freq_odom = config_["simulation"]["frequency"]["odom"].as<mjtNum>();
  timers_.emplace_back(this->create_wall_timer(
      std::chrono::duration<mjtNum, std::milli>{1000.0 / freq_odom},
      std::bind(&SimPublisher::odomCallback, this)));

  mjtNum freq_drop_old_message =
      config_["simulation"]["frequency"]["drop_old_message"].as<mjtNum>();
  timers_.emplace_back(this->create_wall_timer(
      std::chrono::duration<mjtNum, std::milli>{1000.0 / freq_drop_old_message},
      std::bind(&SimPublisher::drop_old_message, this)));
  // timers_.emplace_back(this->create_wall_timer(
  //     1ms, std::bind(&SimPublisher::add_external_disturbance, this)));

  std::string actuators_cmds_topic =
      config_["global"]["topic_names"]["actuators_cmds"].as<std::string>();
  actuator_cmd_subscription_ =
      this->create_subscription<trans::msg::ActuatorCmds>(
          name_prefix + actuators_cmds_topic, qos,
          std::bind(&SimPublisher::actuator_cmd_callback, this,
                    std::placeholders::_1));

  actuator_cmds_buffer_ = std::make_shared<ActuatorCmdsBuffer>();

  RCLCPP_INFO(rclcpp::get_logger("SimPublisher"), "Start SimPublisher ...");
}

SimPublisher::~SimPublisher() {
  RCLCPP_INFO(rclcpp::get_logger("SimPublisher"), "close SimPublisher node ...");
}

void SimPublisher::reset_callback(
    const std::shared_ptr<trans::srv::SimulationReset::Request> request,
    std::shared_ptr<trans::srv::SimulationReset::Response> response) {
  while (sim_->d_ == nullptr && rclcpp::ok()) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  if (sim_->d_ != nullptr) {
    if (request->header.frame_id != std::string(&sim_->m_->names[0])) {
      RCLCPP_ERROR(rclcpp::get_logger("SimPublisher"), "reset request is not for %s",
                   &sim_->m_->names[0]);
      response->is_success = false;
    } else {
      {
        const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);
        mj_resetData(sim_->m_, sim_->d_);
        sim_->d_->qpos[0] = request->base_pose.position.x;
        sim_->d_->qpos[1] = request->base_pose.position.y;
        sim_->d_->qpos[2] = request->base_pose.position.z;
        sim_->d_->qpos[3] = request->base_pose.orientation.w;
        sim_->d_->qpos[4] = request->base_pose.orientation.x;
        sim_->d_->qpos[5] = request->base_pose.orientation.y;
        sim_->d_->qpos[6] = request->base_pose.orientation.z;

        for (size_t i = 0; i < request->joint_state.position.size(); i++) {
          int joint_id = mj_name2id(sim_->m_, mjOBJ_JOINT,
                                    request->joint_state.name[i].c_str());
          if (joint_id > -1) {
            sim_->d_->qpos[sim_->m_->jnt_qposadr[joint_id]] =
                request->joint_state.position[i];
          } else {
            RCLCPP_WARN(rclcpp::get_logger("SimPublisher"),
                        "[Reset Request] joint %s does not exist",
                        request->joint_state.name[i].c_str());
          }
        }
        for (size_t k = 0; k < actuator_cmds_buffer_->actuators_name.size();
             k++) {
          actuator_cmds_buffer_->kp[k] = 0;
          actuator_cmds_buffer_->pos[k] = 0;
          actuator_cmds_buffer_->kd[k] = 0;
          actuator_cmds_buffer_->vel[k] = 0;
          actuator_cmds_buffer_->torque[k] = 0.0;
        }
      }
      response->is_success = true;
      RCLCPP_INFO(rclcpp::get_logger("SimPublisher"), "reset robot state...");
    }
  } else {
    response->is_success = false;
  }
}

void SimPublisher::imuCallback() {
  if (sim_->d_ != nullptr) {
    auto message = sensor_msgs::msg::Imu();
    message.header.frame_id = &sim_->m_->names[0];
    message.header.stamp = rclcpp::Clock().now();
    {
      const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);
      bool acc_flag = true;
      bool gyro_flag = true;
      bool quat_flag = true;
      for (int i = 0; i < sim_->m_->nsensor; i++) {
        if (sim_->m_->sensor_type[i] == mjtSensor::mjSENS_ACCELEROMETER) {
          message.linear_acceleration.x =
              sim_->d_->sensordata[sim_->m_->sensor_adr[i]] +
              noise_acc * mju_standardNormal(nullptr);
          message.linear_acceleration.y =
              sim_->d_->sensordata[sim_->m_->sensor_adr[i] + 1] +
              noise_acc * mju_standardNormal(nullptr);
          message.linear_acceleration.z =
              sim_->d_->sensordata[sim_->m_->sensor_adr[i] + 2] +
              noise_acc * mju_standardNormal(nullptr);
          acc_flag = false;
        } else if (sim_->m_->sensor_type[i] == mjtSensor::mjSENS_FRAMEQUAT) {
          message.orientation.w = sim_->d_->sensordata[sim_->m_->sensor_adr[i]];
          message.orientation.x =
              sim_->d_->sensordata[sim_->m_->sensor_adr[i] + 1];
          message.orientation.y =
              sim_->d_->sensordata[sim_->m_->sensor_adr[i] + 2];
          message.orientation.z =
              sim_->d_->sensordata[sim_->m_->sensor_adr[i] + 3];
          quat_flag = false;
        } else if (sim_->m_->sensor_type[i] == mjtSensor::mjSENS_GYRO) {
          message.angular_velocity.x =
              sim_->d_->sensordata[sim_->m_->sensor_adr[i]] +
              noise_gyro * mju_standardNormal(nullptr);
          message.angular_velocity.y =
              sim_->d_->sensordata[sim_->m_->sensor_adr[i] + 1] +
              noise_gyro * mju_standardNormal(nullptr);
          message.angular_velocity.z =
              sim_->d_->sensordata[sim_->m_->sensor_adr[i] + 2] +
              noise_gyro * mju_standardNormal(nullptr);
          gyro_flag = false;
        }
      }
      if (acc_flag) {
        RCLCPP_WARN(rclcpp::get_logger("SimPublisher"), "Required acc sensor does not exist");
      }
      if (quat_flag) {
        RCLCPP_WARN(rclcpp::get_logger("SimPublisher"), "Required quat sensor does not exist");
      }
      if (gyro_flag) {
        RCLCPP_WARN(rclcpp::get_logger("SimPublisher"), "Required gyro sensor does not exist");
      }
    }
    imu_publisher_->publish(message);
  }
}

void SimPublisher::odomCallback() {
  if (sim_->d_ != nullptr) {
    auto message = nav_msgs::msg::Odometry();
    {
      const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);
      message.header.frame_id = &sim_->m_->names[0];
      message.header.stamp = rclcpp::Clock().now();
      message.pose.pose.position.x = sim_->d_->qpos[0];
      message.pose.pose.position.y = sim_->d_->qpos[1];
      message.pose.pose.position.z = sim_->d_->qpos[2];
      message.pose.pose.orientation.w = sim_->d_->qpos[3];
      message.pose.pose.orientation.x = sim_->d_->qpos[4];
      message.pose.pose.orientation.y = sim_->d_->qpos[5];
      message.pose.pose.orientation.z = sim_->d_->qpos[6];
      message.twist.twist.linear.x = sim_->d_->qvel[0];
      message.twist.twist.linear.y = sim_->d_->qvel[1];
      message.twist.twist.linear.z = sim_->d_->qvel[2];
      message.twist.twist.angular.x = sim_->d_->qvel[3];
      message.twist.twist.angular.y = sim_->d_->qvel[4];
      message.twist.twist.angular.z = sim_->d_->qvel[5];
    }
    odom_publisher_->publish(message);
  }
}

void SimPublisher::jointCallback() {
  if (sim_->d_ != nullptr) {
    sensor_msgs::msg::JointState jointState;
    jointState.header.frame_id = &sim_->m_->names[0];
    jointState.header.stamp = rclcpp::Clock().now();
    {
      const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);
      for (int i = 0; i < sim_->m_->njnt; i++) {
        if (sim_->m_->jnt_type[i] != mjtJoint::mjJNT_FREE) {
          std::string jnt_name(mj_id2name(sim_->m_, mjtObj::mjOBJ_JOINT, i));
          jointState.name.emplace_back(jnt_name);
          jointState.position.push_back(
              sim_->d_->qpos[sim_->m_->jnt_qposadr[i]]);
          jointState.velocity.push_back(
              sim_->d_->qvel[sim_->m_->jnt_dofadr[i]] +
              noise_joint_vel * mju_standardNormal(nullptr));
          jointState.effort.push_back(
              sim_->d_->qfrc_actuator[sim_->m_->jnt_dofadr[i]]);
        }
      }
    }
    // mju_printMat(sim_->d_->qvel, 1, sim_->m_->nv);
    joint_state_publisher_->publish(jointState);
  }
}

void SimPublisher::actuator_cmd_callback(
    const trans::msg::ActuatorCmds::SharedPtr msg) const {
  if (sim_->d_ != nullptr) {
    const std::lock_guard<std::mutex> lock(actuator_cmds_buffer_->mtx);

    actuator_cmds_buffer_->time = rclcpp::Time(msg->header.stamp).seconds();
    actuator_cmds_buffer_->actuators_name.resize(msg->names.size());
    actuator_cmds_buffer_->kp.resize(msg->gain_p.size());
    actuator_cmds_buffer_->pos.resize(msg->pos_des.size());
    actuator_cmds_buffer_->kd.resize(msg->gaid_d.size());
    actuator_cmds_buffer_->vel.resize(msg->vel_des.size());
    actuator_cmds_buffer_->torque.resize(msg->feedforward_torque.size());
    for (size_t k = 0; k < msg->names.size(); k++) {
      actuator_cmds_buffer_->actuators_name[k] = msg->names[k];
      actuator_cmds_buffer_->kp[k] = msg->gain_p[k];
      actuator_cmds_buffer_->pos[k] = msg->pos_des[k];
      actuator_cmds_buffer_->kd[k] = msg->gaid_d[k];
      actuator_cmds_buffer_->vel[k] = msg->vel_des[k];
      actuator_cmds_buffer_->torque[k] = msg->feedforward_torque[k];
    }
    // RCLCPP_INFO(rclcpp::get_logger("SimPublisher"), "subscribe actuator cmds %f",
    // actuator_cmds_buffer_->time);
  }
}

void SimPublisher::drop_old_message() {
  const std::lock_guard<std::mutex> lock(actuator_cmds_buffer_->mtx);
  if (abs(actuator_cmds_buffer_->time - this->now().seconds()) > 0.2) {
    for (size_t k = 0; k < actuator_cmds_buffer_->actuators_name.size(); k++) {
      actuator_cmds_buffer_->kp[k] = 0.0;
      actuator_cmds_buffer_->pos[k] = 0.0;
      actuator_cmds_buffer_->kd[k] = 1.0;
      actuator_cmds_buffer_->vel[k] = 0.0;
      actuator_cmds_buffer_->torque[k] = 0.0;
    }
  }
}

bool added = false;
void SimPublisher::add_external_disturbance() {
  const std::unique_lock<std::recursive_mutex> lock(sim_->mtx);
  if (sim_->d_ == nullptr)
    return;
  if (sim_->d_->time < 15.0) {
    added = false;
    return;
  } else if (added) {
    return;
  } else {
    added = true;
    sim_->d_->qvel[0] += 0.3;
    sim_->d_->qvel[1] += 0.5;
  }

  // int nq = sim_->m_->nq - 1;
  // int nv = sim_->m_->nv - 1;
  // int nq_shift = 0;
  // int nv_shift = 0;

  // if (sim_->d_->time < 5.0) {
  //   return;
  // }
  // for (int i = 0; i < 4; i++) {
  //   std::vector<mjtNum> pos;
  //   std::vector<mjtNum> vel;

  //   switch (i) {
  //   case 0:
  //     pos = {0.45, 0, 0.5};
  //     vel = {0, 0, -1.5};
  //     break;

  //   case 1:
  //     pos = {0.15, -0.5, 0.2};
  //     vel = {0, 2.5, 0};
  //     break;

  //   case 2:
  //     pos = {-0.15, 0.5, 0.2};
  //     vel = {0, -2.5, 0};
  //     break;

  //   case 3:
  //     pos = {0.5, 0.5, 0.5};
  //     vel = {-2.0, -2.0, -2.0};
  //     break;

  //   default:
  //     break;
  //   }
  //   sim_->d_->qpos[nq - nq_shift] = 0;
  //   sim_->d_->qpos[nq - 1 - nq_shift] = 0;
  //   sim_->d_->qpos[nq - 2 - nq_shift] = 0;
  //   sim_->d_->qpos[nq - 3 - nq_shift] = 1;
  //   sim_->d_->qpos[nq - 4 - nq_shift] = sim_->d_->qpos[2] + pos[2];
  //   sim_->d_->qpos[nq - 5 - nq_shift] = sim_->d_->qpos[1] + pos[1];
  //   sim_->d_->qpos[nq - 6 - nq_shift] = sim_->d_->qpos[0] + pos[0];

  //   sim_->d_->qvel[nv - nv_shift] = 0;
  //   sim_->d_->qvel[nv - 1 - nv_shift] = 0;
  //   sim_->d_->qvel[nv - 2 - nv_shift] = 0;
  //   sim_->d_->qvel[nv - 3 - nv_shift] = sim_->d_->qvel[2] + vel[2];
  //   sim_->d_->qvel[nv - 4 - nv_shift] = sim_->d_->qvel[1] + vel[1];
  //   sim_->d_->qvel[nv - 5 - nv_shift] = sim_->d_->qvel[0] + vel[0];

  //   nq_shift += 7;
  //   nv_shift += 6;
  // }
}

std::shared_ptr<ActuatorCmdsBuffer> SimPublisher::get_cmds_buffer() {
  return actuator_cmds_buffer_;
}

} // namespace clear
