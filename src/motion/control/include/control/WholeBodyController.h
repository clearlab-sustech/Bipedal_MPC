#pragma once

#include "control/MatrixDB.h"
#include <core/gait/ModeSchedule.h>
#include <core/gait/MotionPhaseDefinition.h>
#include <core/misc/Buffer.h>
#include <core/trajectory/ReferenceBuffer.h>
#include <pinocchio/PinocchioInterface.h>
#include <rclcpp/rclcpp.hpp>

using namespace rclcpp;

namespace clear {

using namespace biped;

/**
 * @brief 执行器命令结构体 / Actuator Command Structure
 * 
 * 包含发送给机器人关节执行器的所有控制命令
 * Contains all control commands sent to robot joint actuators
 */
struct ActuatorCommands {
  vector_t Kp;      // 比例增益 / Proportional gains
  vector_t Kd;      // 微分增益 / Derivative gains
  vector_t pos;     // 期望位置 / Desired positions
  vector_t vel;     // 期望速度 / Desired velocities
  vector_t torque;  // 期望力矩 / Desired torques

  /**
   * @brief 将所有命令置零 / Set all commands to zero
   * @param n 关节数量 / Number of joints
   */
  void setZero(size_t n) {
    Kp.setZero(n);
    Kd.setZero(n);
    pos.setZero(n);
    vel.setZero(n);
    torque.setZero(n);
  }
};

/**
 * @brief 全身控制器类 / Whole Body Controller Class
 * 
 * 实现基于二次规划的全身控制器，用于双足机器人的运动控制。
 * 决策变量定义为: x = [v̇^T, F^T, τ^T]^T
 * - v̇: 机器人加速度 / Robot acceleration
 * - F: 接触力 / Contact forces  
 * - τ: 关节力矩 / Joint torques
 * 
 * Implements a quadratic programming-based whole body controller for bipedal robot motion control.
 * Decision variables are defined as: x = [v̇^T, F^T, τ^T]^T
 * - v̇: Robot acceleration
 * - F: Contact forces
 * - τ: Joint torques
 */
class WholeBodyController {
public:
  /**
   * @brief 构造函数 / Constructor
   * @param nodeHandle ROS2节点句柄 / ROS2 node handle
   */
  WholeBodyController(Node::SharedPtr nodeHandle);

  ~WholeBodyController();

  /**
   * @brief 加载任务设置参数 / Load task setting parameters
   * @param verbose 是否输出详细信息 / Whether to output verbose information
   */
  void loadTasksSetting(bool verbose = true);

  /**
   * @brief 更新参考轨迹缓冲区 / Update reference trajectory buffer
   * @param referenceBuffer 参考轨迹缓冲区指针 / Reference trajectory buffer pointer
   */
  void updateReferenceBuffer(
      std::shared_ptr<ReferenceBuffer> referenceBuffer);

  /**
   * @brief 更新机器人状态 / Update robot state
   * @param qpos_ptr 关节位置指针 / Joint position pointer
   * @param qvel_ptr 关节速度指针 / Joint velocity pointer
   */
  void updateState(const std::shared_ptr<vector_t> qpos_ptr,
                    const std::shared_ptr<vector_t> qvel_ptr);

  /**
   * @brief 执行全身控制优化 / Execute whole body control optimization
   * @return 执行器命令指针 / Actuator commands pointer
   * 
   * 求解二次规划问题获得最优的关节力矩和控制命令
   * Solve quadratic programming problem to obtain optimal joint torques and control commands
   */
  std::shared_ptr<ActuatorCommands> optimize();

private:
  /**
   * @brief 制定控制问题 / Formulate control problem
   * 组织所有约束条件和目标函数，准备求解优化问题
   * Organize all constraints and objective functions, prepare to solve optimization problem
   */
  void formulate();

  /**
   * @brief 更新接触雅可比矩阵 / Update contact Jacobian matrix
   * 计算足端相对于关节空间的雅可比矩阵
   * Calculate foot Jacobian matrix relative to joint space
   */
  void updateContactJacobi();

  /**
   * @brief 获取决策变量数量 / Get number of decision variables
   * @return 决策变量总数 / Total number of decision variables
   */
  size_t getNumDecisionVars() const { return numDecisionVars_; }

  // 各种任务约束公式化函数 / Various task constraint formulation functions
  /**
   * @brief 制定浮动基座欧拉-牛顿方程 / Formulate floating base Euler-Newton equation
   * 描述机器人整体的动力学约束
   * Describe overall robot dynamics constraints
   */
  MatrixDB formulateFloatingBaseEulerNewtonEqu();
  
  /**
   * @brief 制定力矩限制任务 / Formulate torque limits task
   * 确保关节力矩在安全范围内
   * Ensure joint torques are within safe range
   */
  MatrixDB formulateTorqueLimitsTask();
  
  /**
   * @brief 制定维持接触任务 / Formulate maintain contact task
   * 确保接触足端保持零加速度
   * Ensure contact feet maintain zero acceleration
   */
  MatrixDB formulateMaintainContactTask();
  
  /**
   * @brief 制定摩擦锥任务 / Formulate friction cone task
   * 确保接触力满足摩擦约束
   * Ensure contact forces satisfy friction constraints
   */
  MatrixDB formulateFrictionConeTask();
  
  /**
   * @brief 制定基座任务 / Formulate base task
   * 控制机器人基座的位置和姿态跟踪
   * Control robot base position and orientation tracking
   */
  MatrixDB formulateBaseTask();
  
  /**
   * @brief 制定摆动腿任务 / Formulate swing leg task
   * 控制摆动腿的轨迹跟踪
   * Control swing leg trajectory tracking
   */
  MatrixDB formulateSwingLegTask();
  
  /**
   * @brief 制定接触力任务 / Formulate contact force task
   * 跟踪MPC优化得到的期望接触力
   * Track desired contact forces from MPC optimization
   */
  MatrixDB formulateContactForceTask();

  /**
   * @brief 微分逆运动学控制 / Differential inverse kinematics control
   * 将笛卡尔空间的运动转换为关节空间的运动
   * Convert Cartesian space motion to joint space motion
   */
  void differential_inv_kin();

  void simpleCtrl();

private:
  Node::SharedPtr nodeHandle_;                                  // ROS2节点句柄 / ROS2 node handle
  std::shared_ptr<ReferenceBuffer> referenceBuffer_;          // 参考轨迹缓冲区 / Reference trajectory buffer
  Buffer<size_t> mode_;                                        // 步态模式缓冲区 / Gait mode buffer

  size_t numDecisionVars_;                                     // 决策变量数量 / Number of decision variables
  std::shared_ptr<PinocchioInterface> pinocchioInterface_ptr_; // 机器人动力学接口 / Robot dynamics interface

  std::string base_name, robot_name;                          // 基座和机器人名称 / Base and robot names
  std::vector<std::string> foot_names, actuated_joints_name;  // 足端和驱动关节名称 / Foot and actuated joint names

  matrix_t Jc;                                                // 接触雅可比矩阵 / Contact Jacobian matrix
  contact_flag_t contactFlag_{};                             // 接触标志 / Contact flags
  size_t numContacts_{};                                      // 接触点数量 / Number of contacts

  // 任务参数 / Task Parameters:
  matrix_t weightSwingLeg_, weightBase_, weightMomentum_;     // 摆动腿、基座、动量权重 / Swing leg, base, momentum weights
  scalar_t weightContactForce_;                               // 接触力权重 / Contact force weight
  matrix_t swingKp_, swingKd_, baseKp_, baseKd_, momentumKp_, momentumKd_; // PD控制增益 / PD control gains
  scalar_t frictionCoeff_{};                                  // 摩擦系数 / Friction coefficient

  MatrixDB weighedTask, constraints;                          // 加权任务和约束 / Weighted tasks and constraints
  scalar_t dt_ = 0.002;                                       // 控制时间步长 / Control time step
  vector_t joint_acc_;                                        // 关节加速度 / Joint accelerations
  std::shared_ptr<ActuatorCommands> actuator_commands_;       // 执行器命令 / Actuator commands
};
} // namespace clear