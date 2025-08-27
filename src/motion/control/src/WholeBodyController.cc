#include "control/WholeBodyController.h"

#include <pinocchio/Orientation.h>
#include <yaml-cpp/yaml.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <eiquadprog/eiquadprog-fast.hpp>
#include <rcpputils/asserts.hpp>
#include <utility>

namespace clear {

/**
 * @brief WholeBodyController构造函数 / WholeBodyController Constructor
 * 
 * 初始化全身控制器，加载机器人模型和配置参数
 * Initialize whole body controller, load robot model and configuration parameters
 */
WholeBodyController::WholeBodyController(Node::SharedPtr nodeHandle)
    : nodeHandle_(nodeHandle) {
  // 获取配置文件路径 / Get configuration file path
  const std::string config_file_ = nodeHandle_->get_parameter("/config_file")
                                       .get_parameter_value()
                                       .get<std::string>();

  auto config_ = YAML::LoadFile(config_file_);

  // 加载机器人URDF模型 / Load robot URDF model
  std::string model_package = config_["model"]["package"].as<std::string>();
  std::string urdf =
      ament_index_cpp::get_package_share_directory(model_package) +
      config_["model"]["urdf"].as<std::string>();
  RCLCPP_INFO(rclcpp::get_logger("WholeBodyController"), "model file: %s",
              urdf.c_str());
  pinocchioInterface_ptr_ = std::make_shared<PinocchioInterface>(urdf.c_str());

  // 获取足端名称列表 / Get foot names list
  foot_names = config_["model"]["foot_names"].as<std::vector<std::string>>();
  for (const auto &name : foot_names) {
    RCLCPP_INFO(rclcpp::get_logger("WholeBodyController"), "foot name: %s",
                name.c_str());
  }

  // 获取基座名称 / Get base name
  base_name = config_["model"]["base_name"].as<std::string>();

  // 获取驱动关节名称列表 / Get actuated joint names list
  actuated_joints_name =
      config_["model"]["actuated_joints_name"].as<std::vector<std::string>>();

  // 计算决策变量总数 / Calculate total number of decision variables
  // 决策变量 = 机器人速度维度 + 接触力维度 + 关节力矩维度
  // Decision variables = robot velocity dimension + contact force dimension + joint torque dimension
  numDecisionVars_ = pinocchioInterface_ptr_->nv() + 3 * foot_names.size() +
                     actuated_joints_name.size();
  
  // 加载任务设置参数 / Load task setting parameters
  this->loadTasksSetting(false);
}

WholeBodyController::~WholeBodyController() {}

/**
 * @brief 更新参考轨迹缓冲区 / Update reference trajectory buffer
 * @param referenceBuffer 参考轨迹缓冲区指针 / Reference trajectory buffer pointer
 */
void WholeBodyController::updateReferenceBuffer(
    std::shared_ptr<ReferenceBuffer> referenceBuffer) {
  referenceBuffer_ = referenceBuffer;
}
/**
 * @brief 更新机器人状态 / Update robot state
 * 
 * 使用最新的关节位置和速度更新机器人动力学模型
 * Update robot dynamics model with latest joint positions and velocities
 */
void WholeBodyController::updateState(
    const std::shared_ptr<vector_t> qpos_ptr,
    const std::shared_ptr<vector_t> qvel_ptr) {
  pinocchioInterface_ptr_->updateRobotState(*qpos_ptr, *qvel_ptr);
}

/**
 * @brief 制定优化问题 / Formulate optimization problem
 * 
 * 组织所有的约束条件和目标函数，为二次规划求解做准备
 * Organize all constraints and objective functions, prepare for quadratic programming solving
 */
void WholeBodyController::formulate() {
  // 获取当前步态模式 / Get current gait mode
  mode_.push(referenceBuffer_->getModeSchedule()->getModeFromPhase(0.0));
  contactFlag_ = modeNumber2StanceLeg(mode_.get());
  numContacts_ = std::count(contactFlag_.cbegin(), contactFlag_.cend(), true);

  // 更新接触雅可比矩阵 / Update contact Jacobian matrix
  updateContactJacobi();

  // 组合所有约束条件 / Combine all constraints
  // 包括：动力学约束、力矩限制、摩擦锥约束、接触约束
  // Including: dynamics constraints, torque limits, friction cone constraints, contact constraints
  constraints = formulateFloatingBaseEulerNewtonEqu() +
                formulateTorqueLimitsTask() + formulateFrictionConeTask() +
                formulateMaintainContactTask();
  
  // 组合所有任务目标 / Combine all task objectives
  // 包括：基座控制、摆动腿控制、接触力跟踪
  // Including: base control, swing leg control, contact force tracking
  weighedTask = formulateBaseTask() + formulateSwingLegTask() +
                formulateContactForceTask();
}

/**
 * @brief 执行全身控制优化 / Execute whole body control optimization
 * @return 执行器命令指针 / Actuator commands pointer
 * 
 * 求解二次规划问题，获得最优的关节控制命令
 * Solve quadratic programming problem to obtain optimal joint control commands
 */
std::shared_ptr<ActuatorCommands> WholeBodyController::optimize() {
  // 初始化执行器命令 / Initialize actuator commands
  actuator_commands_ = std::make_shared<ActuatorCommands>();
  actuator_commands_->setZero(actuated_joints_name.size());

  // 检查必要的参考轨迹是否可用 / Check if necessary reference trajectories are available
  if (referenceBuffer_->getModeSchedule().get() == nullptr ||
      referenceBuffer_->getOptimizedForceTraj() == nullptr ||
      referenceBuffer_->getFootPosTraj().empty()) {
    return actuator_commands_;
  }

  // 制定优化问题 / Formulate optimization problem
  formulate();

  // 构建二次规划问题的Hessian矩阵和梯度向量 / Build Hessian matrix and gradient vector for QP problem
  matrix_t H = weighedTask.A.transpose() * weighedTask.A;  // H = A^T * A
  H.diagonal() += 1e-12 * vector_t::Ones(numDecisionVars_); // 添加正则化项防止奇异 / Add regularization to prevent singularity
  vector_t g = -weighedTask.A.transpose() * weighedTask.b; // g = -A^T * b

  // 初始化二次规划求解器 / Initialize quadratic programming solver
  eiquadprog::solvers::EiquadprogFast eiquadprog_solver;
  eiquadprog_solver.reset(numDecisionVars_, constraints.b.size(),
                          2 * constraints.lb.size());
  
  // 设置不等式约束 / Set up inequality constraints
  // 将单边约束转换为双边约束形式 / Convert one-sided constraints to two-sided form
  matrix_t Cin(constraints.C.rows() * 2, numDecisionVars_);
  Cin << constraints.C, -constraints.C;
  vector_t cin(constraints.C.rows() * 2), ce0;
  cin << -constraints.lb, constraints.ub;
  ce0 = -constraints.b;
  
  // 执行二次规划求解 / Execute quadratic programming solving
  vector_t optimal_u = vector_t::Zero(numDecisionVars_);
  auto solver_state = eiquadprog_solver.solve_quadprog(H, g, constraints.A, ce0,
                                                       Cin, cin, optimal_u);
  
  // 处理求解结果 / Process solving results
  if (solver_state == eiquadprog::solvers::EIQUADPROG_FAST_OPTIMAL) {
    // 提取最优关节力矩 / Extract optimal joint torques
    actuator_commands_->torque = optimal_u.tail(actuated_joints_name.size());
    
    // 提取关节加速度用于微分逆运动学 / Extract joint accelerations for differential inverse kinematics
    joint_acc_ = optimal_u.head(pinocchioInterface_ptr_->nv())
                     .tail(actuated_joints_name.size());
    
    // 执行微分逆运动学计算位置控制命令 / Execute differential inverse kinematics for position control commands
    differential_inv_kin();

    /* 调试信息：打印基座加速度 / Debug info: print base acceleration
    matrix6x_t Jbase;
    pinocchioInterface_ptr_->getJacobia_local(base_name, Jbase);
    std::cout << "base acc opt: " << (Jbase *
    optimal_u.head(pinocchioInterface_ptr_->nv())).transpose() << "\n"; */
  } else {
    // 求解失败时的处理 / Handle solving failure
    joint_acc_.setZero(actuated_joints_name.size());
    std::cerr << "wbc failed ...\n";
    actuator_commands_->setZero(actuated_joints_name.size());
    actuator_commands_->Kd.fill(1.0);
  }

  return actuator_commands_;
}

/**
 * @brief 简单控制模式 / Simple Control Mode
 * 
 * 实现一个简化的控制策略，用于基本的力/位置控制
 * 支撑腿使用位置保持，摆动腿使用轨迹跟踪
 * 
 * Implement a simplified control strategy for basic force/position control
 * Stance legs use position holding, swing legs use trajectory tracking
 */
void WholeBodyController::simpleCtrl() {
  const scalar_t t_now = nodeHandle_->now().seconds();

  // 计算关节力矩：将足端接触力映射到关节空间 / Calculate joint torques: map foot contact forces to joint space
  // τ = -J^T * F + τ_gravity_compensation
  actuator_commands_->torque =
      -Jc.block(0, 6, 6, 6).transpose() *
          referenceBuffer_->getOptimizedForceTraj()->evaluate(t_now) +
      pinocchioInterface_ptr_->nle().tail(6);  // 重力补偿项 / Gravity compensation term

  // 为每条腿设置控制命令 / Set control commands for each leg
  for (int i = 0; i < 2; i++) {
    if (contactFlag_[i])  // 支撑腿模式 / Stance leg mode
    {
      // 支撑腿保持当前位置，关闭PD控制 / Stance leg holds current position, disable PD control
      actuator_commands_->Kp.segment(3 * i, 3).setZero();
      actuator_commands_->Kd.segment(3 * i, 3).setZero();
      actuator_commands_->pos.segment(3 * i, 3) =
          pinocchioInterface_ptr_->qpos().segment(3 * i + 7, 3);
      actuator_commands_->vel.segment(3 * i, 3).setZero();
    } else  // 摆动腿模式 / Swing leg mode
    {
      // 获取足端轨迹和基座速度轨迹 / Get foot trajectory and base velocity trajectory
      auto foot_traj = referenceBuffer_->getFootPosTraj();
      auto base_vel_traj = referenceBuffer_->getOptimizedBaseVelTraj();

      // 获取当前足端位置 / Get current foot position
      vector3_t foot_pos =
          pinocchioInterface_ptr_->getFramePose(foot_names[i]).translation();

      // 设置摆动腿PD控制增益 / Set swing leg PD control gains
      actuator_commands_->Kp.segment(3 * i, 3) << 30, 30, 30;
      actuator_commands_->Kd.segment(3 * i, 3) << 8.0, 8.0, 8.0;

      // 通过逆雅可比计算关节空间的运动 / Calculate joint space motion through inverse Jacobian
      matrix3_t J_inv = Jc.block<3, 3>(3 * i, i * 3 + 6).inverse();
      vector3_t delta_q =
          J_inv * (foot_traj[foot_names[i]]->evaluate(t_now) - foot_pos);
      vector3_t qd_des =
          J_inv * (foot_traj[foot_names[i]]->derivative(t_now, 1) -
                   base_vel_traj->evaluate(t_now));

      // 设置期望关节位置和速度 / Set desired joint positions and velocities
      actuator_commands_->pos.segment(3 * i, 3) =
          pinocchioInterface_ptr_->qpos().segment(3 * i + 7, 3) + delta_q;
      actuator_commands_->vel.segment(3 * i, 3) = qd_des;
    }
  }
}


/**
 * @brief 更新接触雅可比矩阵 / Update contact Jacobian matrix
 * 
 * 计算所有足端相对于机器人关节的雅可比矩阵，用于将笛卡尔空间的
 * 足端运动映射到关节空间
 * 
 * Calculate Jacobian matrices of all feet relative to robot joints,
 * used to map Cartesian space foot motion to joint space
 */
void WholeBodyController::updateContactJacobi() {
  // 初始化接触雅可比矩阵 / Initialize contact Jacobian matrix
  Jc = matrix_t(3 * foot_names.size(), pinocchioInterface_ptr_->nv());
  
  // 为每个足端计算雅可比矩阵 / Calculate Jacobian matrix for each foot
  for (size_t i = 0; i < foot_names.size(); ++i) {
    matrix6x_t jac;  // 6D雅可比矩阵（线性+角度） / 6D Jacobian matrix (linear + angular)
    pinocchioInterface_ptr_->getJacobia_localWorldAligned(foot_names[i], jac);
    Jc.middleRows(3 * i, 3) = jac.topRows(3);  // 只取线性部分 / Only take linear part
  }
}

/**
 * @brief 制定浮动基座欧拉-牛顿方程约束 / Formulate floating base Euler-Newton equation constraints
 * 
 * 构建机器人整体动力学约束方程：M*a + h = S*τ + J^T*F
 * 其中：M是惯性矩阵，a是加速度，h是科里奥利力和重力项，
 * S是选择矩阵，τ是关节力矩，J是雅可比矩阵，F是接触力
 * 
 * Build robot overall dynamics constraint equation: M*a + h = S*τ + J^T*F
 * Where: M is inertia matrix, a is acceleration, h is Coriolis and gravity terms,
 * S is selection matrix, τ is joint torques, J is Jacobian matrix, F is contact forces
 */
MatrixDB WholeBodyController::formulateFloatingBaseEulerNewtonEqu() {
  MatrixDB eulerNewtonEqu("eulerNewtonEqu");
  auto &data = pinocchioInterface_ptr_->getData();
  size_t nv = pinocchioInterface_ptr_->nv();    // 速度维度 / Velocity dimension
  size_t na = actuated_joints_name.size();     // 驱动关节数 / Number of actuated joints

  // 检查维度一致性 / Check dimension consistency
  if (nv != na + 6) {
    throw std::runtime_error("nv != info_.actuatedDofNum + 6");
  }
  
  // 构建选择矩阵S，将关节力矩映射到广义力空间 / Build selection matrix S, map joint torques to generalized force space
  matrix_t S(nv, na);
  S.topRows(6).setZero();      // 浮动基座部分为零 / Floating base part is zero
  S.bottomRows(na).setIdentity(); // 驱动关节部分为单位矩阵 / Actuated joint part is identity matrix
  
  // 构建约束矩阵：[M, -J^T, -S] * [a; F; τ] = -h
  // Build constraint matrix: [M, -J^T, -S] * [a; F; τ] = -h
  eulerNewtonEqu.A =
      (matrix_t(nv, numDecisionVars_) << data.M, -Jc.transpose(), -S)
          .finished();
  eulerNewtonEqu.b = -data.nle;  // -h：负的非线性效应项 / -h: negative nonlinear effects term
  return eulerNewtonEqu;
}

/**
 * @brief 制定力矩限制约束 / Formulate torque limits constraints
 * 
 * 为驱动关节施加力矩上下限约束：τ_min ≤ τ ≤ τ_max
 * 这确保生成的关节力矩在电机能力范围内
 * 
 * Apply upper and lower torque limits for actuated joints: τ_min ≤ τ ≤ τ_max
 * This ensures generated joint torques are within motor capability range
 */
MatrixDB WholeBodyController::formulateTorqueLimitsTask() {
  MatrixDB limit_tau("limit_tau");
  size_t na = actuated_joints_name.size();  // 驱动关节数 / Number of actuated joints
  
  // 构建约束矩阵，提取力矩部分的决策变量（右下角na×na块）
  // Build constraint matrix, extract torque part of decision variables (bottom-right na×na block)
  limit_tau.C.setZero(na, numDecisionVars_);
  limit_tau.C.bottomRightCorner(na, na).setIdentity();
  
  // 设置力矩限制边界（从机器人模型获取努力限制）
  // Set torque limit boundaries (get effort limits from robot model)
  limit_tau.lb = -pinocchioInterface_ptr_->getModel().effortLimit.tail(na);  // 下限 / Lower bound
  limit_tau.ub = pinocchioInterface_ptr_->getModel().effortLimit.tail(na);   // 上限 / Upper bound
  return limit_tau;
}

/**
 * @brief 制定接触保持约束 / Formulate maintain contact constraints
 * 
 * 为接触脚建立运动学约束，确保接触脚的加速度为零：J*a = -J̇*q̇
 * 这保证了接触脚在地面上保持静止不动
 * 
 * Establish kinematic constraints for contact feet, ensuring contact foot acceleration is zero: J*a = -J̇*q̇
 * This guarantees that contact feet remain stationary on the ground
 */
MatrixDB WholeBodyController::formulateMaintainContactTask() {
  MatrixDB contact_task("contact_task");
  size_t nc = foot_names.size();                    // 足端数量 / Number of feet
  size_t nv = pinocchioInterface_ptr_->nv();       // 速度维度 / Velocity dimension
  
  // 初始化约束矩阵和向量 / Initialize constraint matrix and vector
  contact_task.A.setZero(3 * numContacts_, numDecisionVars_);
  contact_task.b.setZero(3 * numContacts_);
  
  // 为每个接触脚建立约束 / Establish constraints for each contact foot
  size_t j = 0;  // 当前接触脚索引 / Current contact foot index
  for (size_t i = 0; i < nc; i++) {
    if (contactFlag_[i]) {  // 如果该脚处于接触状态 / If this foot is in contact
      // 提取该脚的雅可比矩阵（线性部分） / Extract Jacobian matrix for this foot (linear part)
      contact_task.A.block(3 * j, 0, 3, nv) = Jc.middleRows(3 * i, 3);
      
      // 设置约束右端项：-J̇*q̇（负的脚端加速度）/ Set constraint RHS: -J̇*q̇ (negative foot acceleration)
      contact_task.b.segment(3 * j, 3) =
          -pinocchioInterface_ptr_
               ->getFrame6dAcc_localWorldAligned(foot_names[i])
               .linear();
      j++;
    }
  }
  return contact_task;
}

/**
 * @brief 制定摩擦锥约束 / Formulate friction cone constraints
 * 
 * 建立接触力的摩擦锥约束，包括：
 * 1. 非接触脚的接触力为零
 * 2. 接触脚的摩擦锥限制：|F_x|, |F_y| ≤ μ*F_z, F_z ≥ 0
 * 
 * Establish friction cone constraints for contact forces, including:
 * 1. Zero contact force for non-contact feet
 * 2. Friction cone limits for contact feet: |F_x|, |F_y| ≤ μ*F_z, F_z ≥ 0
 */
MatrixDB WholeBodyController::formulateFrictionConeTask() {
  MatrixDB friction_cone("friction_cone");
  size_t nc = foot_names.size();                    // 足端数量 / Number of feet
  size_t nv = pinocchioInterface_ptr_->nv();       // 速度维度 / Velocity dimension
  size_t j = 0;

  // 为非接触脚设置零力约束 / Set zero force constraints for non-contact feet
  friction_cone.A.setZero(3 * (nc - numContacts_), numDecisionVars_);
  for (size_t i = 0; i < nc; ++i) {
    if (!contactFlag_[i]) {  // 非接触脚 / Non-contact foot
      friction_cone.A.block(3 * j++, nv + 3 * i, 3, 3) =
          matrix_t::Identity(3, 3);
    }
  }
  friction_cone.b.setZero(friction_cone.A.rows());

  // 定义摩擦锥矩阵：5个不等式约束
  // Define friction cone matrix: 5 inequality constraints
  // [F_z ≥ 0, F_x - μ*F_z ≤ 0, -F_x - μ*F_z ≤ 0, F_y - μ*F_z ≤ 0, -F_y - μ*F_z ≤ 0]
  matrix_t frictionPyramic(5, 3);  // clang-format off
  frictionPyramic << 0, 0, 1,                    // F_z ≥ 0
                     1, 0, -frictionCoeff_,      // F_x ≤ μ*F_z
                    -1, 0, -frictionCoeff_,      // -F_x ≤ μ*F_z
                     0, 1, -frictionCoeff_,      // F_y ≤ μ*F_z
                     0,-1, -frictionCoeff_;      // -F_y ≤ μ*F_z
  // clang-format on
  
  // 为接触脚设置摩擦锥约束 / Set friction cone constraints for contact feet
  friction_cone.C.setZero(5 * numContacts_, numDecisionVars_);
  friction_cone.ub = Eigen::VectorXd::Zero(friction_cone.C.rows());     // 上界为0 / Upper bound is 0
  friction_cone.lb = -1e16 * Eigen::VectorXd::Ones(friction_cone.C.rows()); // 下界为负无穷 / Lower bound is negative infinity

  j = 0;
  for (size_t i = 0; i < nc; ++i) {
    if (contactFlag_[i]) {  // 接触脚 / Contact foot
      friction_cone.C.block(5 * j, nv + 3 * i, 5, 3) = frictionPyramic;
      friction_cone.lb(5 * j) = 0.0;    // F_z下界为0 / F_z lower bound is 0
      friction_cone.ub(5 * j) = 400.0;  // F_z上界为400N / F_z upper bound is 400N
      j++;
    }
  }
  return friction_cone;
}

/**
 * @brief 制定基座任务 / Formulate base task
 * 
 * 建立机器人基座的跟踪任务，包括位置、姿态和速度跟踪
 * 使用反馈控制律：acc_fb = Kp*pose_err + Kd*vel_err + acc_ref
 * 
 * Establish robot base tracking task, including position, orientation and velocity tracking
 * Using feedback control law: acc_fb = Kp*pose_err + Kd*vel_err + acc_ref
 */
MatrixDB WholeBodyController::formulateBaseTask() {
  MatrixDB base_task("base_task");
  size_t nv = pinocchioInterface_ptr_->nv();       // 速度维度 / Velocity dimension

  // 构建基座雅可比矩阵 / Build base Jacobian matrix
  base_task.A.setZero(6, numDecisionVars_);
  matrix6x_t J = matrix6x_t::Zero(6, nv);
  pinocchioInterface_ptr_->getJacobia_local(base_name, J);
  base_task.A.leftCols(nv) = J;

  vector6_t acc_fb;  // 反馈加速度 / Feedback acceleration
  
  // 获取参考轨迹 / Get reference trajectories
  auto pos_traj = referenceBuffer_.get()->getIntegratedBasePosTraj();     // 位置轨迹 / Position trajectory
  auto rpy_traj = referenceBuffer_.get()->getIntegratedBaseRpyTraj();     // 姿态轨迹 / Orientation trajectory
  auto vel_traj = referenceBuffer_.get()->getOptimizedBaseVelTraj();      // 速度轨迹 / Velocity trajectory
  auto omega_traj = referenceBuffer_.get()->getOptimizedBaseOmegaTraj();  // 角速度轨迹 / Angular velocity trajectory

  // 如果所有参考轨迹都有效，计算反馈控制 / If all reference trajectories are valid, compute feedback control
  if (pos_traj.get() != nullptr && rpy_traj.get() != nullptr &&
      vel_traj.get() != nullptr && omega_traj.get() != nullptr) {
    scalar_t time_now_ = nodeHandle_->now().seconds() + dt_;
    
    // 获取当前基座状态 / Get current base state
    auto base_pose = pinocchioInterface_ptr_->getFramePose(base_name);
    auto base_twist = pinocchioInterface_ptr_->getFrame6dVel_local(base_name);
    
    // 构建参考姿态 / Build reference pose
    pin::SE3 pose_ref;
    pose_ref.rotation() = toRotationMatrix(rpy_traj->evaluate(time_now_));
    pose_ref.translation() = pos_traj->evaluate(time_now_);
    
    // 构建参考速度和加速度（转换到局部坐标系）/ Build reference velocity and acceleration (convert to local frame)
    vector6_t _spatialVelRef, _spatialAccRef;
    _spatialVelRef << base_pose.rotation().transpose() * vel_traj->evaluate(time_now_),
                      base_pose.rotation().transpose() * omega_traj->evaluate(time_now_);
    _spatialAccRef << base_pose.rotation().transpose() * vel_traj->derivative(time_now_, 1),
                      base_pose.rotation().transpose() * omega_traj->derivative(time_now_, 1);
    
    // 计算误差 / Compute errors
    auto pose_err = log6(base_pose.actInv(pose_ref)).toVector();  // 位姿误差 / Pose error
    auto vel_err = _spatialVelRef - base_twist.toVector();        // 速度误差 / Velocity error

    // 反馈控制律 / Feedback control law
    acc_fb = baseKp_ * pose_err + baseKd_ * vel_err + _spatialAccRef;
  } else {
    acc_fb.setZero();  // 无参考轨迹时设为零 / Set to zero when no reference trajectory
  }

  // 计算任务向量：期望加速度减去当前加速度 / Compute task vector: desired acceleration minus current acceleration
  base_task.b = acc_fb - pinocchioInterface_ptr_->getFrame6dAcc_local(base_name).toVector();

  // 应用权重 / Apply weights
  base_task.A = weightBase_ * base_task.A;
  base_task.b = weightBase_ * base_task.b;

  return base_task;
}

/**
 * @brief 制定摆动腿任务 / Formulate swing leg task
 * 
 * 为非接触（摆动）脚建立轨迹跟踪任务，使用反馈控制跟踪期望轨迹
 * 控制律：acc_fb = Kp*pos_err + Kd*vel_err + acc_des
 * 
 * Establish trajectory tracking task for non-contact (swing) feet, using feedback control to track desired trajectory
 * Control law: acc_fb = Kp*pos_err + Kd*vel_err + acc_des
 */
MatrixDB WholeBodyController::formulateSwingLegTask() {
  const size_t nc = foot_names.size();            // 足端数量 / Number of feet
  const size_t nv = pinocchioInterface_ptr_->nv(); // 速度维度 / Velocity dimension

  // 获取足端轨迹和基座位置轨迹 / Get foot trajectories and base position trajectory
  auto foot_traj = referenceBuffer_->getFootPosTraj();
  auto base_pos_traj = referenceBuffer_->getOptimizedBasePosTraj();

  if (nc - numContacts_ <= 0 || foot_traj.size() != nc ||
      base_pos_traj.get() == nullptr) {
    return MatrixDB("swing_task");
  }
  MatrixDB swing_task("swing_task");
  scalar_t t = nodeHandle_->now().seconds() + dt_;

  matrix_t Qw =
      matrix_t::Zero(3 * (nc - numContacts_), 3 * (nc - numContacts_));
  swing_task.A.setZero(3 * (nc - numContacts_), numDecisionVars_);
  swing_task.b.setZero(swing_task.A.rows());

  size_t j = 0;
  for (size_t i = 0; i < nc; ++i) {
    const auto &foot_name = foot_names[i];
    if (!contactFlag_[i]) {
      Qw.block<3, 3>(3 * j, 3 * j) = weightSwingLeg_;
      const auto traj = foot_traj[foot_name];
      vector3_t pos_des = traj->evaluate(t);
      vector3_t vel_des = traj->derivative(t, 1);
      vector3_t acc_des = traj->derivative(t, 2);
      vector3_t pos_m =
          pinocchioInterface_ptr_->getFramePose(foot_name).translation();
      // pos_m.z() -= 0.03;
      vector3_t vel_m =
          pinocchioInterface_ptr_->getFrame6dVel_localWorldAligned(foot_name)
              .linear();
      vector3_t pos_err = pos_des - pos_m;
      vector3_t vel_err = vel_des - vel_m;
      vector3_t accel_fb = swingKp_ * pos_err + swingKd_ * vel_err;
      /* if (accel_fb.norm() > 10.0) {
        accel_fb = 10.0 * accel_fb.normalized();
      } */
      swing_task.A.block(3 * j, 0, 3, nv) = Jc.block(3 * i, 0, 3, nv);
      swing_task.b.segment(3 * j, 3) =
          -pinocchioInterface_ptr_->getFrame6dAcc_localWorldAligned(foot_name)
               .linear() +
          accel_fb + acc_des;
      j++;
    }
  }
  // log_stream << swing_task.b.transpose() << std::endl;
  swing_task.A.leftCols(6).setZero();
  swing_task.A = Qw * swing_task.A;
  swing_task.b = Qw * swing_task.b;
  return swing_task;
}

/**
 * @brief 微分逆运动学求解 / Differential inverse kinematics solver
 * 
 * 通过微分逆运动学计算关节角度命令，用于实现足端轨迹跟踪
 * 求解：q̇ = J^(-1) * ẋ_des，其中J是雅可比矩阵，ẋ_des是期望足端速度
 * 
 * Calculate joint angle commands through differential inverse kinematics for foot trajectory tracking
 * Solve: q̇ = J^(-1) * ẋ_des, where J is Jacobian matrix, ẋ_des is desired foot velocity
 */
void WholeBodyController::differential_inv_kin() {
  // 获取足端轨迹和基座位置轨迹 / Get foot trajectories and base position trajectory
  auto foot_traj_array = referenceBuffer_->getFootPosTraj();
  auto pos_traj = referenceBuffer_.get()->getOptimizedBasePosTraj();

  // 检查轨迹有效性 / Check trajectory validity
  if (foot_traj_array.empty() || pos_traj.get() == nullptr) {
    return;
  }

  int nj = static_cast<int>(actuated_joints_name.size());  // 关节数量 / Number of joints
  size_t nf = foot_names.size();                          // 足端数量 / Number of feet
  auto contact_flag = biped::modeNumber2StanceLeg(mode_.get());  // 接触标志 / Contact flags
  scalar_t time_c = nodeHandle_->now().seconds() + 0.002;  // 当前时间 / Current time
  
  // 获取基座当前状态 / Get current base state
  auto base_pose = pinocchioInterface_ptr_->getFramePose(base_name);
  auto base_twist = pinocchioInterface_ptr_->getFrame6dVel_localWorldAligned(base_name);

  // 为每个足端计算逆运动学 / Compute inverse kinematics for each foot
  for (size_t k = 0; k < nf; k++) {
    const auto &foot_name = foot_names[k];
    matrix6x_t Jac_k;  // 足端雅可比矩阵 / Foot Jacobian matrix
    pinocchioInterface_ptr_->getJacobia_localWorldAligned(foot_name, Jac_k);
    vector<int> idx;
    for (int i = 0; i < nj; i++) {
      if (Jac_k.col(i + 6).head(3).norm() > 0.01) {
        idx.emplace_back(i);
      }
    }
    rcpputils::assert_true(idx.size() == 3);

    if (!contact_flag[k]) {
      const auto foot_traj = foot_traj_array[foot_name];
      matrix3_t Js_;
      vector3_t qpos_s;

      for (size_t i = 0; i < 3; i++) {
        Js_.col(i) = Jac_k.col(idx[i] + 6).head(3);
        qpos_s(i) = pinocchioInterface_ptr_->qpos()(7 + idx[i]);
      }
      matrix3_t J_inv = Js_.inverse();

      vector3_t pos_des, vel_des;
      vector3_t pos_m, vel_m;
      pos_m = (pinocchioInterface_ptr_->getFramePose(foot_name).translation() -
               base_pose.translation());
      vel_m =
          (pinocchioInterface_ptr_->getFrame6dVel_localWorldAligned(foot_name)
               .linear() -
           base_twist.linear());

      /* if (pos_traj.get() == nullptr) {
        pos_des = (foot_traj->evaluate(time_c) - base_pose.translation());
        vel_des = (foot_traj->derivative(time_c, 1) - base_twist.linear());
      } else {
        pos_des = (foot_traj->evaluate(time_c) - pos_traj->evaluate(time_c));
        vel_des = (foot_traj->derivative(time_c, 1) -
                   pos_traj->derivative(time_c, 1));
      } */
      pos_des = (foot_traj->evaluate(time_c) - base_pose.translation());
      vel_des = (foot_traj->derivative(time_c, 1) - base_twist.linear());

      vector3_t pos_err = (pos_des - pos_m);
      if (pos_err.norm() > 0.1) {
        pos_err = 0.1 * pos_err.normalized();
      }
      vector3_t vel_err = (vel_des - vel_m);
      if (vel_err.norm() > 0.5) {
        vel_des = 0.5 * vel_err.normalized() + vel_m;
      }

      scalar_t kp_val = 40.0;
      scalar_t kd_val = 2.0;

      // scalar_t kp_val, kd_val;
      // if (pos_err.norm() < 3e-2) {
      //   kp_val = 60;
      // } else {
      //   kp_val = 30;
      // }
      // if (vel_err.norm() < 0.1) {
      //   kd_val = 2;
      // } else {
      //   kd_val = 1.0;
      // }
      vector3_t q_des = J_inv * pos_err + qpos_s;
      vector3_t qd_des = J_inv * vel_des;
      for (size_t i = 0; i < 3; i++) {
        actuator_commands_->Kp(idx[i]) = kp_val;
        actuator_commands_->Kd(idx[i]) = kd_val;
        actuator_commands_->pos(idx[i]) = q_des(i);
        actuator_commands_->vel(idx[i]) = qd_des(i);
      }
    } else {
      if (joint_acc_.size() == nj) {
        vector_t jnt_pos = pinocchioInterface_ptr_->qpos().tail(nj);
        vector_t jnt_vel = pinocchioInterface_ptr_->qvel().tail(nj);
        for (size_t i = 0; i < 3; i++) {
          actuator_commands_->Kp(idx[i]) = 10.0;
          actuator_commands_->Kd(idx[i]) = 0.1;
          actuator_commands_->pos(idx[i]) =
              jnt_pos(idx[i]) + jnt_vel(idx[i]) * dt_ +
              0.5 * pow(dt_, 2) * joint_acc_(idx[i]);
          actuator_commands_->vel(idx[i]) =
              jnt_vel(idx[i]) + dt_ * joint_acc_(idx[i]);
          // actuator_commands_->Kp(idx[i]) = 0.0;
          // actuator_commands_->Kd(idx[i]) = 0.0;
          // actuator_commands_->pos(idx[i]) = 0.0;
          // actuator_commands_->vel(idx[i]) = 0.0;
        }
      }
    }
  }
  /* std::cout << "#####################################################\n";
  std::cout << "jnt kp: " <<  actuator_commands_->Kp.transpose() << "\n";
  std::cout << "jnt kd: " <<  actuator_commands_->Kd.transpose() << "\n"; */
}

/**
 * @brief 制定接触力任务 / Formulate contact force task
 * 
 * 建立接触力跟踪任务，使实际接触力跟踪MPC优化得到的参考接触力
 * 这有助于保持控制一致性和改善跟踪性能
 * 
 * Establish contact force tracking task to make actual contact forces track reference forces from MPC optimization
 * This helps maintain control consistency and improve tracking performance
 */
MatrixDB WholeBodyController::formulateContactForceTask() {
  size_t nc = foot_names.size();                   // 足端数量 / Number of feet
  size_t nv = pinocchioInterface_ptr_->nv();      // 速度维度 / Velocity dimension

  MatrixDB contact_force("contact_force");
  
  // 构建接触力任务矩阵 / Build contact force task matrix
  contact_force.A.setZero(3 * nc, numDecisionVars_);
  
  // 获取MPC优化的参考接触力 / Get reference contact forces from MPC optimization
  contact_force.b = referenceBuffer_->getOptimizedForceTraj()->evaluate(
      nodeHandle_->now().seconds());
  
  weightContactForce_ = 50.0;  // 接触力任务权重 / Contact force task weight
  
  // 为每个足端设置接触力约束（提取决策变量中的力部分）
  // Set contact force constraints for each foot (extract force part from decision variables)
  for (size_t i = 0; i < nc; ++i) {
    contact_force.A.block<3, 3>(3 * i, nv + 3 * i) = matrix_t::Identity(3, 3);
  }
  contact_force.A = weightContactForce_ * contact_force.A;
  contact_force.b = weightContactForce_ * contact_force.b;
  return contact_force;
}

/**
 * @brief 加载任务设置参数 / Load task setting parameters
 * 
 * 设置WBC中各个任务的权重矩阵和反馈控制增益
 * 包括基座跟踪、摆动腿控制、接触力跟踪、摩擦系数等参数
 * 
 * Set weight matrices and feedback control gains for various tasks in WBC
 * Including base tracking, swing leg control, contact force tracking, friction coefficient, etc.
 * 
 * @param verbose 是否打印参数信息 / Whether to print parameter information
 */
void WholeBodyController::loadTasksSetting(bool verbose) {
  // 初始化权重矩阵 / Initialize weight matrices
  
  // 动量任务权重（当前未使用）/ Momentum task weight (currently unused)
  weightMomentum_.setZero(6, 6);
  
  // 基座跟踪权重：线性和角度跟踪 / Base tracking weight: linear and angular tracking
  weightBase_.setZero(6, 6);
  weightBase_.diagonal().fill(100);

  // 摆动腿跟踪权重：xyz方向 / Swing leg tracking weight: xyz directions
  weightSwingLeg_.setZero(3, 3);
  weightSwingLeg_.diagonal().fill(200);

  // 接触力跟踪权重 / Contact force tracking weight
  weightContactForce_ = 1e-2;

  // 摩擦系数设置 / Friction coefficient setting
  frictionCoeff_ = 0.5;

  // 摆动腿比例增益：位置误差反馈 / Swing leg proportional gain: position error feedback
  swingKp_.setZero(3, 3);
  swingKp_.diagonal().fill(350);

  // 摆动腿微分增益：速度误差反馈 / Swing leg derivative gain: velocity error feedback
  swingKd_.setZero(3, 3);
  swingKd_.diagonal().fill(37);

  // 基座比例增益：[x, y, z, roll, pitch, yaw] / Base proportional gain: [x, y, z, roll, pitch, yaw]
  baseKp_.setZero(6, 6);
  baseKp_.diagonal() << 30, 30, 60, 80, 80, 80;    // 配置1 / Configuration 1
  // baseKp_.diagonal() << 60, 60, 100, 120, 120, 120;  // 配置2 / Configuration 2
  baseKp_.diagonal() << 200, 200, 200, 320, 320, 320;  // 配置3（当前使用）/ Configuration 3 (currently used)

  // 基座微分增益：[x, y, z, roll, pitch, yaw] / Base derivative gain: [x, y, z, roll, pitch, yaw]
  baseKd_.setZero(6, 6);
  baseKd_.diagonal() << 3.0, 3.0, 3.0, 10.0, 10.0, 10.0;    // 配置1 / Configuration 1
  // baseKd_.diagonal() << 16.0, 16.0, 20.0, 20.0, 20.0, 20.0;  // 配置2 / Configuration 2
  baseKd_.diagonal() << 16.0, 16.0, 20.0, 40.0, 40.0, 40.0;    // 配置3（当前使用）/ Configuration 3 (currently used)

  // 动量反馈增益（当前未使用）/ Momentum feedback gains (currently unused)
  momentumKp_.setZero(6, 6);
  momentumKp_.diagonal().fill(0);

  momentumKd_.setZero(6, 6);
  momentumKd_.diagonal().fill(0);

  // 如果需要详细输出，打印所有参数 / If verbose output needed, print all parameters
  if (verbose) {
    std::cerr << "\n ########### weights.momentum ########### \n";
    std::cerr << weightMomentum_ << "\n";
    std::cerr << "\n ########### weights.floatingbase ########### \n";
    std::cerr << weightBase_ << "\n";
    std::cerr << "\n ########### weights.leg_swing ########### \n";
    std::cerr << weightSwingLeg_ << "\n";
    std::cerr << "\n ########### weights.weightContactForce_: "
              << weightContactForce_ << "\n";
    std::cerr << "\n ########### friction_coefficient: " << frictionCoeff_
              << "\n";

    std::cerr << "\n ########### feedback_gain.leg_swing.kp ########### \n";
    std::cerr << swingKp_ << "\n";
    std::cerr << "\n ########### feedback_gain.leg_swing.kd ########### \n";
    std::cerr << swingKd_ << "\n";
    std::cerr << "\n ########### feedback_gain.floatingbase.kp ########### \n";
    std::cerr << baseKp_ << "\n";
    std::cerr << "\n ########### feedback_gain.floatingbase.kd ########### \n";
    std::cerr << baseKd_ << "\n";
    std::cerr << "\n ########### feedback_gain.momentum.kp ########### \n";
    std::cerr << momentumKp_ << "\n";
    std::cerr << "\n ########### feedback_gain.momentum.kd ########### \n";
    std::cerr << momentumKd_ << "\n";
  }
}
}  // namespace clear