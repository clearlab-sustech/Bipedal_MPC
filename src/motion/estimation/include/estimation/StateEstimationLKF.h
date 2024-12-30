#pragma once
#include <core/gait/ModeSchedule.h>
#include <core/gait/MotionPhaseDefinition.h>
#include <core/misc/Buffer.h>
#include <core/types.h>
#include <nav_msgs/msg/odometry.hpp>
#include <pinocchio/PinocchioInterface.h>
#include <rclcpp/rclcpp.hpp>
#include <rmw/types.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using namespace rclcpp;
using namespace std::chrono_literals;

namespace clear {
class StateEstimationLKF {
public:
  StateEstimationLKF(Node::SharedPtr nodeHandle);

  ~StateEstimationLKF();

  std::shared_ptr<vector_t> getQpos();

  std::shared_ptr<vector_t> getQvel();

  void setContactFlag(vector<bool> flag);

  void setImuMsg(sensor_msgs::msg::Imu::SharedPtr msg);

  void setJointsMsg(sensor_msgs::msg::JointState::SharedPtr msg);

  void updateModeSchedule(std::shared_ptr<ModeSchedule> mode_schedule);

private:
  void setup();

  void angularMotionEstimate(const sensor_msgs::msg::Imu &imu_data,
                             std::shared_ptr<vector_t> qpos,
                             std::shared_ptr<vector_t> qvel);

  void linearMotionEstimate(const sensor_msgs::msg::Imu &imu_data,
                            std::shared_ptr<vector_t> qpos,
                            std::shared_ptr<vector_t> qvel);

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) const;

  void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg) const;

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) const;

  void innerLoop();

private:
  Node::SharedPtr nodeHandle_;
  std::unique_ptr<PinocchioInterface> pinocchioInterface_ptr;
  std::string robot_name;
  std::vector<string> foot_names;
  vector<bool> cflag_;
  Buffer<std::shared_ptr<ModeSchedule>> mode_schedule_buffer_;

  Buffer<std::shared_ptr<vector_t>> qpos_ptr_buffer, qvel_ptr_buffer;
  mutable Buffer<sensor_msgs::msg::Imu::SharedPtr> imu_msg_buffer;
  mutable Buffer<sensor_msgs::msg::JointState::SharedPtr>
      joint_state_msg_buffer;
  mutable Buffer<nav_msgs::msg::Odometry::SharedPtr> odom_msg_buffer;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joints_state_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_est_publisher_;

  std::thread inner_loop_thread_;
  Buffer<bool> run_;
  bool use_odom_ = false;

  scalar_t dt_;
  Eigen::Matrix<scalar_t, 6, 1> ps_;
  Eigen::Matrix<scalar_t, 6, 1> vs_;
  Eigen::Matrix<scalar_t, 12, 12> A_;
  Eigen::Matrix<scalar_t, 12, 3> B_;
  Eigen::Matrix<scalar_t, 14, 12> C_;
  Eigen::Matrix<scalar_t, 12, 12> Sigma_;
  Eigen::Matrix<scalar_t, 12, 12> Q0_;
  Eigen::Matrix<scalar_t, 14, 14> R0_;
  Eigen::Matrix<scalar_t, 12, 1> x_est;
};
} // namespace clear