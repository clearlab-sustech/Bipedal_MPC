#pragma once

#include "DataVisualization.h"
#include "Initialization.h"
#include "JoyStick.h"
#include <control/TrajectoryStabilization.h>
#include <estimation/StateEstimationLKF.h>
#include <gait/GaitSchedule.h>
#include <generation/TrajectorGeneration.h>
#include <rclcpp/rclcpp.hpp>
#include <string>

using namespace rclcpp;

namespace clear {

/**
 * @brief 运动管理器类 / Motion Manager Class
 * 
 * 这是系统的核心管理类，负责协调和管理双足机器人运动控制的各个模块，
 * 包括状态估计、步态规划、轨迹生成、控制和可视化等。
 * 
 * This is the core management class of the system, responsible for coordinating
 * and managing various modules of bipedal robot motion control, including state
 * estimation, gait planning, trajectory generation, control, and visualization.
 */
class MotionManager : public Node {

public:
  /**
   * @brief 构造函数 / Constructor
   * 初始化运动管理器并创建所有子模块
   * Initialize motion manager and create all sub-modules
   */
  MotionManager();

  /**
   * @brief 析构函数 / Destructor
   * 清理资源并停止所有线程
   * Clean up resources and stop all threads
   */
  ~MotionManager();

  /**
   * @brief 初始化系统 / Initialize system
   * 启动所有模块并开始运动控制主循环
   * Start all modules and begin main motion control loop
   */
  void init();

private:
  /**
   * @brief 内部控制循环 / Internal control loop
   * 高频率运行的主控制循环，协调各模块的执行
   * High-frequency main control loop that coordinates module execution
   */
  void innerLoop();

private:
  // 各个功能模块指针 / Pointers to functional modules
  std::shared_ptr<StateEstimationLKF> estimatorPtr_;           // 状态估计器 / State estimator
  std::shared_ptr<GaitSchedule> gaitSchedulePtr_;              // 步态调度器 / Gait scheduler
  std::shared_ptr<TrajectorGeneration> trajGenPtr_;            // 轨迹生成器 / Trajectory generator
  std::shared_ptr<TrajectoryStabilization> trajectoryStabilizationPtr_; // 轨迹稳定化控制器 / Trajectory stabilization controller
  std::shared_ptr<DataVisualization> visPtr_;                 // 数据可视化 / Data visualization
  std::shared_ptr<Initialization> intializationPtr_;          // 初始化模块 / Initialization module
  std::shared_ptr<JoyStick> joyStickPtr_;                     // 手柄控制 / Joystick control

  std::thread inner_loop_thread_;                             // 内部循环线程 / Internal loop thread
  Buffer<bool> run_;                                          // 运行状态缓冲区 / Run state buffer
};

} // namespace clear
