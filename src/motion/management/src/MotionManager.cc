#include <MotionManager.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <core/misc/Benchmark.h>
#include <pinocchio/Orientation.h>
#include <yaml-cpp/yaml.h>

namespace clear {

/**
 * @brief MotionManager构造函数 / MotionManager Constructor
 * 
 * 初始化ROS2节点并声明配置文件参数
 * Initialize ROS2 node and declare configuration file parameter
 */
MotionManager::MotionManager() : Node("MotionManager") {
  // 声明配置文件参数 / Declare configuration file parameter
  this->declare_parameter("/config_file", "");
}

/**
 * @brief MotionManager析构函数 / MotionManager Destructor
 * 
 * 停止内部循环线程并进行资源清理
 * Stop internal loop thread and perform resource cleanup
 */
MotionManager::~MotionManager() {
  run_.push(false);              // 设置停止标志 / Set stop flag
  inner_loop_thread_.join();     // 等待内部线程结束 / Wait for internal thread to finish
}

/**
 * @brief 系统初始化函数 / System Initialization Function
 * 
 * 按顺序初始化所有子系统模块，包括状态估计、步态规划、轨迹生成、控制器等，
 * 并启动高频控制循环线程。
 * 
 * Initialize all subsystem modules in order, including state estimation,
 * gait planning, trajectory generation, controller, etc., and start the
 * high-frequency control loop thread.
 */
void MotionManager::init() {
  // 初始化系统初始化模块 / Initialize system initialization module
  intializationPtr_ =
      std::make_shared<Initialization>(this->shared_from_this());

  // 初始化手柄控制模块 / Initialize joystick control module
  joyStickPtr_ = std::make_shared<JoyStick>(this->shared_from_this());

  // 等待手柄启动（可选）/ Wait for joystick startup (optional)
  // while (!joyStickPtr_->isStart()) {
  //   rclcpp::spin_some(this->shared_from_this());
  // }

  // 初始化状态估计器 / Initialize state estimator
  estimatorPtr_ =
      std::make_shared<StateEstimationLKF>(this->shared_from_this());

  RCLCPP_INFO(this->get_logger(), "StateEstimationLKF Ready");

  // 初始化步态调度器 / Initialize gait scheduler
  gaitSchedulePtr_ = std::make_shared<GaitSchedule>(this->shared_from_this());

  RCLCPP_INFO(this->get_logger(), "GaitSchedule Ready");

  // 初始化轨迹生成器 / Initialize trajectory generator
  trajGenPtr_ = std::make_shared<TrajectorGeneration>(this->shared_from_this());

  RCLCPP_INFO(this->get_logger(), "TrajectorGeneration Ready");

  // 初始化轨迹稳定化控制器 / Initialize trajectory stabilization controller
  trajectoryStabilizationPtr_ =
      std::make_shared<TrajectoryStabilization>(this->shared_from_this());

  // 初始化数据可视化模块 / Initialize data visualization module
  visPtr_ = std::make_shared<DataVisualization>(this->shared_from_this());

  // 加载配置文件 / Load configuration file
  const std::string config_file_ = this->get_parameter("/config_file")
                                       .get_parameter_value()
                                       .get<std::string>();

  auto config_ = YAML::LoadFile(config_file_);
  
  // 启动高频控制循环线程 / Start high-frequency control loop thread
  inner_loop_thread_ = std::thread(&MotionManager::innerLoop, this);
  run_.push(true);  // 设置运行标志 / Set running flag
}

/**
 * @brief 内部高频控制循环 / Internal High-Frequency Control Loop
 * 
 * 这是系统的核心控制循环，以1000Hz的频率运行，协调各个模块的执行：
 * 1. 检查紧急停止状态
 * 2. 处理用户速度命令
 * 3. 更新步态调度
 * 4. 状态估计更新
 * 5. 轨迹生成和优化
 * 6. 控制器执行
 * 7. 数据可视化更新
 * 
 * This is the core control loop of the system, running at 1000Hz frequency,
 * coordinating the execution of various modules:
 * 1. Check emergency stop status
 * 2. Process user velocity commands
 * 3. Update gait scheduling
 * 4. Update state estimation
 * 5. Trajectory generation and optimization
 * 6. Controller execution
 * 7. Data visualization update
 */
void MotionManager::innerLoop() {
  // 初始延时，等待系统稳定 / Initial delay for system stabilization
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  // 设置循环频率为1000Hz / Set loop frequency to 1000Hz
  rclcpp::Rate loop_rate(1000.0);
  const scalar_t ts = this->now().seconds();

  // 主控制循环 / Main control loop
  while (rclcpp::ok() && run_.get()) {

    // 检查紧急停止状态 / Check emergency stop status
    if (joyStickPtr_->eStop()) {
      RCLCPP_INFO(this->get_logger(), "e-stop");
      // 安全关闭相关模块 / Safely shutdown related modules
      trajGenPtr_.reset();
      trajectoryStabilizationPtr_.reset();
      visPtr_.reset();
      break;
    }

    // 处理行走模式下的速度命令 / Process velocity commands in walk mode
    if (gaitSchedulePtr_->getCurrentGaitName() == "walk") {
      trajGenPtr_->setVelCmd(joyStickPtr_->getLinearVelCmd(),
                             joyStickPtr_->getYawVelCmd());
    }

    // 计算预测时域长度，根据步态周期动态调整 / Calculate prediction horizon based on gait cycle
    scalar_t horizon_time_ =
        min(2.0, max(0.5, gaitSchedulePtr_->currentGaitCycle()));

    // 生成步态调度 / Generate gait schedule
    auto mode_schedule_ptr = gaitSchedulePtr_->eval(horizon_time_);

    // 更新状态估计器的步态信息 / Update state estimator with gait information
    estimatorPtr_->updateModeSchedule(mode_schedule_ptr);

    // 更新轨迹生成器的当前状态 / Update trajectory generator with current state
    trajGenPtr_->updateCurrentState(estimatorPtr_->getQpos(),
                                    estimatorPtr_->getQvel());

    // 更新轨迹生成器的步态调度 / Update trajectory generator with gait schedule
    trajGenPtr_->updateModeSchedule(mode_schedule_ptr);

    // 更新控制器的当前状态 / Update controller with current state
    trajectoryStabilizationPtr_->updateCurrentState(estimatorPtr_->getQpos(),
                                                    estimatorPtr_->getQvel());

    // 更新控制器的参考轨迹 / Update controller with reference trajectory
    trajectoryStabilizationPtr_->updateReferenceBuffer(
        trajGenPtr_->getReferenceBuffer());

    // 更新可视化模块的状态信息 / Update visualization module with state information
    visPtr_->updateCurrentState(estimatorPtr_->getQpos(),
                                estimatorPtr_->getQvel());
    visPtr_->updateReferenceBuffer(trajGenPtr_->getReferenceBuffer());

    // 控制循环频率 / Control loop frequency
    loop_rate.sleep();
  }

  // 退出程序 / Exit program
  exit(0);
}

} // namespace clear
