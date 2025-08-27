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

/**
 * @brief 基于线性卡尔曼滤波的状态估计器类 / Linear Kalman Filter-based State Estimator Class
 * 
 * 该类实现了双足机器人的状态估计，融合IMU数据、关节编码器数据和接触信息
 * 来估计机器人的位置、速度、姿态等状态信息。
 * 
 * This class implements state estimation for bipedal robots, fusing IMU data,
 * joint encoder data, and contact information to estimate robot position,
 * velocity, orientation, and other state information.
 */
class StateEstimationLKF {
public:
  /**
   * @brief 构造函数 / Constructor
   * @param nodeHandle ROS2节点句柄 / ROS2 node handle
   */
  StateEstimationLKF(Node::SharedPtr nodeHandle);

  ~StateEstimationLKF();

  /**
   * @brief 获取关节位置 / Get joint positions
   * @return 关节位置向量指针 / Joint position vector pointer
   */
  std::shared_ptr<vector_t> getQpos();

  /**
   * @brief 获取关节速度 / Get joint velocities
   * @return 关节速度向量指针 / Joint velocity vector pointer
   */
  std::shared_ptr<vector_t> getQvel();

  /**
   * @brief 设置接触标志 / Set contact flags
   * @param flag 接触标志向量 / Contact flag vector
   */
  void setContactFlag(vector<bool> flag);

  /**
   * @brief 设置IMU消息 / Set IMU message
   * @param msg IMU消息指针 / IMU message pointer
   */
  void setImuMsg(sensor_msgs::msg::Imu::SharedPtr msg);

  /**
   * @brief 设置关节消息 / Set joint message
   * @param msg 关节状态消息指针 / Joint state message pointer
   */
  void setJointsMsg(sensor_msgs::msg::JointState::SharedPtr msg);

  /**
   * @brief 更新步态调度 / Update mode schedule
   * @param mode_schedule 步态调度指针 / Mode schedule pointer
   */
  void updateModeSchedule(std::shared_ptr<ModeSchedule> mode_schedule);

private:
  /**
   * @brief 初始化设置 / Setup initialization
   */
  void setup();

  /**
   * @brief 角运动估计 / Angular motion estimation
   * @param imu_data IMU数据 / IMU data
   * @param qpos 关节位置 / Joint positions
   * @param qvel 关节速度 / Joint velocities
   * 
   * 使用IMU数据估计机器人的姿态角和角速度
   * Estimate robot orientation and angular velocity using IMU data
   */
  void angularMotionEstimate(const sensor_msgs::msg::Imu &imu_data,
                             std::shared_ptr<vector_t> qpos,
                             std::shared_ptr<vector_t> qvel);

  /**
   * @brief 线性运动估计 / Linear motion estimation
   * @param imu_data IMU数据 / IMU data
   * @param qpos 关节位置 / Joint positions
   * @param qvel 关节速度 / Joint velocities
   * 
   * 融合IMU加速度和足端接触信息估计机器人的位置和线速度
   * Fuse IMU acceleration and foot contact information to estimate robot position and linear velocity
   */
  void linearMotionEstimate(const sensor_msgs::msg::Imu &imu_data,
                            std::shared_ptr<vector_t> qpos,
                            std::shared_ptr<vector_t> qvel);

  // ROS2回调函数 / ROS2 callback functions
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