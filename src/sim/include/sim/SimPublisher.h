#pragma once
// reference: https://github.com/MindSpaceInc/Spot-MuJoCo-ROS2

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmw/types.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <simulate/array_safety.h>
#include <simulate/simulate.h>

#include <trans/msg/actuator_cmds.hpp>
#include <trans/msg/touch_sensor.hpp>
#include <trans/srv/simulation_reset.hpp>

using namespace rclcpp;

using namespace std::chrono_literals;

namespace clear {
namespace mj = ::mujoco;
namespace mju = ::mujoco::sample_util;

struct ActuatorCmdsBuffer {
  double time = 0.0;
  std::vector<std::string> actuators_name;
  std::vector<mjtNum> kp;
  std::vector<mjtNum> pos;
  std::vector<mjtNum> kd;
  std::vector<mjtNum> vel;
  std::vector<mjtNum> torque;
  std::mutex mtx;
};

class SimPublisher : public rclcpp::Node {
public:
  SimPublisher(mj::Simulate *sim, const std::string config_yaml);

  ~SimPublisher();

  std::shared_ptr<ActuatorCmdsBuffer> get_cmds_buffer();

private:
  void reset_callback(
      const std::shared_ptr<trans::srv::SimulationReset::Request> request,
      std::shared_ptr<trans::srv::SimulationReset::Response> response);

  void imuCallback();

  void odomCallback();

  void jointCallback();

  void
  actuator_cmd_callback(const trans::msg::ActuatorCmds::SharedPtr msg) const;

  void drop_old_message();

  void add_external_disturbance();

  mj::Simulate *sim_;
  std::vector<rclcpp::TimerBase::SharedPtr> timers_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;

  rclcpp::Subscription<trans::msg::ActuatorCmds>::SharedPtr
      actuator_cmd_subscription_;
  rclcpp::Service<trans::srv::SimulationReset>::SharedPtr reset_service_;

  std::shared_ptr<ActuatorCmdsBuffer> actuator_cmds_buffer_;

  std::thread spin_thread;

  const mjtNum noise_acc = 0.3;
  const mjtNum noise_gyro = 0.05;
  const mjtNum noise_joint_vel = 0.3;

  // const mjtNum noise_acc = 0.0;
  // const mjtNum noise_gyro = 0.0;
  // const mjtNum noise_joint_vel = 0.0;
};

} // namespace clear
