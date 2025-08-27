#pragma once
#include <core/gait/ModeSchedule.h>
#include <core/gait/MotionPhaseDefinition.h>
#include <core/trajectory/ReferenceBuffer.h>

#include <core/misc/Buffer.h>
#include <hpipm-cpp/hpipm-cpp.hpp>
#include <pinocchio/Orientation.h>
#include <pinocchio/PinocchioInterface.h>
#include <rclcpp/rclcpp.hpp>
#include <string>

using namespace rclcpp;

namespace clear {

/**
 * @brief 凸优化模型预测控制器类 / Convex Model Predictive Controller Class
 * 
 * 该类实现了基于凸优化的模型预测控制算法，用于双足机器人的运动控制。
 * 通过将非线性的双足机器人动力学模型转换为凸优化问题来求解最优控制输入。
 * 
 * This class implements a convex optimization-based model predictive control algorithm
 * for bipedal robot motion control. It converts the nonlinear bipedal robot dynamics
 * into a convex optimization problem to solve for optimal control inputs.
 */
class ConvexMPC {
public:
  /**
   * @brief 构造函数 / Constructor
   * @param nodeHandle ROS2节点句柄 / ROS2 node handle
   * @param pinocchioInterface_ptr Pinocchio机器人模型接口 / Pinocchio robot model interface
   * @param referenceBuffer 参考轨迹缓冲区 / Reference trajectory buffer
   */
  ConvexMPC(Node::SharedPtr nodeHandle,
            std::shared_ptr<PinocchioInterface> pinocchioInterface_ptr,
            std::shared_ptr<ReferenceBuffer> referenceBuffer);

  ~ConvexMPC();

  /**
   * @brief 执行MPC优化求解 / Execute MPC optimization
   * 
   * 求解凸优化问题，获得最优的接触力和机器人状态轨迹
   * Solve the convex optimization problem to obtain optimal contact forces and robot state trajectories
   */
  void optimize();

  /**
   * @brief 设置速度命令 / Set velocity command
   * @param vd 期望线速度 / Desired linear velocity
   * @param yawd 期望角速度 / Desired angular velocity
   */
  void setVelCmd(vector3_t vd, scalar_t yawd);

  /**
   * @brief 设置机器人期望高度 / Set desired robot height
   * @param h 期望高度值 / Desired height value
   */
  void setHeightCmd(scalar_t h);

  /**
   * @brief 生成参考轨迹 / Generate reference trajectory
   * 
   * 根据当前状态和速度命令生成机器人的参考运动轨迹
   * Generate robot reference motion trajectory based on current state and velocity commands
   */
  void generateTrajRef();

private:

  /**
   * @brief 获取系统动力学模型 / Get system dynamics model
   * @param time_cur 当前时间 / Current time
   * @param k 预测步长索引 / Prediction horizon index
   * @param mode_schedule 步态调度器 / Gait scheduler
   * 
   * 构建线性化的双足机器人动力学模型用于MPC预测
   * Build linearized bipedal robot dynamics model for MPC prediction
   */
  void getDynamics(scalar_t time_cur, size_t k,
                   const std::shared_ptr<ModeSchedule> mode_schedule);

  /**
   * @brief 获取不等式约束 / Get inequality constraints
   * @param k 预测步长索引 / Prediction horizon index
   * @param N 预测步长总数 / Total prediction horizon
   * @param mode_schedule 步态调度器 / Gait scheduler
   * 
   * 设置摩擦锥约束和接触力约束等不等式约束条件
   * Set up inequality constraints such as friction cone and contact force constraints
   */
  void
  getInequalityConstraints(size_t k, size_t N,
                           const std::shared_ptr<ModeSchedule> mode_schedule);

  /**
   * @brief 获取代价函数 / Get cost function
   * @param time_cur 当前时间 / Current time
   * @param k 预测步长索引 / Prediction horizon index
   * @param N 预测步长总数 / Total prediction horizon
   * @param mode_schedule 步态调度器 / Gait scheduler
   * 
   * 设置MPC优化的代价函数，包括状态跟踪误差和控制输入代价
   * Set up the cost function for MPC optimization, including state tracking error and control input cost
   */
  void getCosts(scalar_t time_cur, size_t k, size_t N,
                const std::shared_ptr<ModeSchedule> mode_schedule);

  /**
   * @brief 拟合轨迹 / Fit trajectory
   * @param time_cur 当前时间 / Current time
   * @param N 预测步长总数 / Total prediction horizon
   * 
   * 将优化结果拟合成平滑的轨迹用于控制器执行
   * Fit optimization results into smooth trajectories for controller execution
   */
  void fitTraj(scalar_t time_cur, size_t N);

private:
  Node::SharedPtr nodeHandle_;                                    // ROS2节点句柄 / ROS2 node handle
  std::shared_ptr<PinocchioInterface> pinocchioInterface_ptr_;   // 机器人动力学模型接口 / Robot dynamics model interface
  std::shared_ptr<ReferenceBuffer> referenceBuffer_;            // 参考轨迹缓冲区 / Reference trajectory buffer

  std::string base_name;                                         // 机器人基座坐标系名称 / Robot base frame name
  std::vector<std::string> foot_names;                          // 足端坐标系名称列表 / Foot frame names list

  scalar_t h_des = 0.68;                                        // 期望机器人高度 / Desired robot height
  scalar_t dt_ = 0.02;                                          // MPC控制时间步长 / MPC control time step
  const scalar_t grav_ = 9.81;                                 // 重力加速度 / Gravitational acceleration
  scalar_t total_mass_;                                         // 机器人总质量 / Total robot mass
  matrix3_t Ig_;                                                // 机器人惯性张量 / Robot inertia tensor
  const scalar_t mu_ = 0.5;                                     // 摩擦系数 / Friction coefficient
  matrix_t weight_;                                             // 状态权重矩阵 / State weight matrix
  vector3_t rpy_des_start;                                      // 初始期望姿态角 / Initial desired orientation angles
  bool first_run_ = true;                                       // 首次运行标志 / First run flag

  std::vector<hpipm::OcpQp> ocp_;                              // 最优控制问题求解器数组 / Optimal control problem solver array
  std::vector<hpipm::OcpQpSolution> solution_;                 // 优化解数组 / Optimization solution array
  hpipm::OcpQpIpmSolverSettings solver_settings;              // 求解器设置 / Solver settings

  vector3_t vel_cmd;                                           // 速度命令 / Velocity command
  scalar_t yawd_;                                              // 偏航角速度命令 / Yaw rate command

  vector3_t rpy_start;                                         // 初始姿态角 / Initial orientation angles
  vector3_t pos_start;                                         // 初始位置 / Initial position
  bool first_run = true;                                       // 首次运行标志 / First run flag
  scalar_t t0 = 0.0;                                           // 起始时间 / Start time
};

} // namespace clear
