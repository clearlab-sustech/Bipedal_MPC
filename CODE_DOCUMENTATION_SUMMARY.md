# 双足机器人MPC控制系统代码注释总结 / Bipedal Robot MPC Control System Code Documentation Summary

## 概述 / Overview

本文档总结了为双足机器人MPC控制系统添加的中英文注释。该系统基于凸优化模型预测控制(Convex MPC)和全身控制(Whole Body Control)实现双足机器人的动态行走控制。

This document summarizes the bilingual (Chinese-English) comments added to the bipedal robot MPC control system. The system implements dynamic walking control for bipedal robots based on Convex Model Predictive Control (MPC) and Whole Body Control (WBC).

## 已注释的主要文件 / Main Annotated Files

### 1. 核心控制类 / Core Control Classes

#### ConvexMPC (凸优化模型预测控制器)
- **文件**: `src/motion/generation/include/generation/ConvexMPC.h`
- **文件**: `src/motion/generation/src/ConvexMPC.cc`
- **功能**: 实现基于凸优化的模型预测控制算法
- **关键方法**:
  - `ConvexMPC()`: 构造函数，初始化求解器参数
  - `setVelCmd()`: 设置速度命令
  - `generateTrajRef()`: 生成参考轨迹
  - `getDynamics()`: 构建线性化动力学模型
  - `optimize()`: 执行MPC优化求解

#### WholeBodyController (全身控制器)
- **文件**: `src/motion/control/include/control/WholeBodyController.h`
- **文件**: `src/motion/control/src/WholeBodyController.cc`
- **功能**: 基于二次规划的全身控制器
- **决策变量**: x = [v̇^T, F^T, τ^T]^T (加速度、接触力、关节力矩)
- **关键方法**:
  - `formulate()`: 制定优化问题
  - `optimize()`: 执行二次规划求解
  - `formulateBaseTask()`: 基座控制任务
  - `formulateSwingLegTask()`: 摆动腿控制任务
  - `differential_inv_kin()`: 微分逆运动学

### 2. 状态估计模块 / State Estimation Module

#### StateEstimationLKF (线性卡尔曼滤波状态估计器)
- **文件**: `src/motion/estimation/include/estimation/StateEstimationLKF.h`
- **功能**: 融合IMU、关节编码器和接触信息进行状态估计
- **关键方法**:
  - `angularMotionEstimate()`: 角运动估计
  - `linearMotionEstimate()`: 线性运动估计

### 3. 运动管理模块 / Motion Management Module

#### MotionManager (运动管理器)
- **文件**: `src/motion/management/include/MotionManager.h`
- **文件**: `src/motion/management/src/MotionManager.cc`
- **功能**: 系统核心管理类，协调各模块执行
- **关键方法**:
  - `MotionManager()`: 构造函数，初始化ROS2节点
  - `init()`: 系统初始化，创建所有子模块
  - `innerLoop()`: 高频控制循环（1000Hz），协调模块执行
- **管理模块**:
  - 状态估计器 (StateEstimationLKF)
  - 步态调度器 (GaitSchedule)
  - 轨迹生成器 (TrajectorGeneration)
  - 控制器 (TrajectoryStabilization)
  - 可视化 (DataVisualization)
  - 手柄控制 (JoyStick)

### 4. 步态定义模块 / Gait Definition Module

#### MotionPhaseDefinition (运动相位定义)
- **文件**: `src/core/include/core/gait/MotionPhaseDefinition.h`
- **功能**: 定义双足机器人步态模式和接触状态
- **步态模式**:
  - FLY: 飞行期（双足离地）
  - LF: 左足支撑期
  - RF: 右足支撑期
  - STANCE: 双足支撑期

#### LegLogic (腿部逻辑)
- **文件**: `src/core/include/core/gait/LegLogic.h`
- **功能**: 处理足端接触时间逻辑
- **关键函数**:
  - `getTimeOfNextTouchDown()`: 获取下次着地时间
  - `getTimeOfNextLiftOff()`: 获取下次抬起时间

### 5. 数据类型定义 / Data Type Definitions

#### Types (核心数据类型)
- **文件**: `src/core/include/core/types.h`
- **功能**: 定义系统中使用的基本数据类型
- **类型定义**:
  - 标量和向量类型
  - 轨迹数据类型
  - Eigen矩阵和向量类型

### 6. 执行器命令结构 / Actuator Command Structure

#### ActuatorCommands (执行器命令)
- **文件**: `src/motion/control/include/control/WholeBodyController.h`
- **功能**: 包含发送给机器人关节执行器的控制命令
- **成员变量**:
  - `Kp`: 比例增益
  - `Kd`: 微分增益
  - `pos`: 期望位置
  - `vel`: 期望速度
  - `torque`: 期望力矩

## 系统架构概述 / System Architecture Overview

```
MotionManager (运动管理器)
├── StateEstimationLKF (状态估计)
├── GaitSchedule (步态调度)
├── TrajectorGeneration (轨迹生成)
│   └── ConvexMPC (凸优化MPC)
├── TrajectoryStabilization (轨迹稳定化)
│   └── WholeBodyController (全身控制)
├── DataVisualization (数据可视化)
└── JoyStick (手柄控制)
```

## 控制流程 / Control Flow

### MotionManager控制循环 / MotionManager Control Loop
MotionManager以1000Hz频率执行以下步骤：
MotionManager executes the following steps at 1000Hz frequency:

1. **紧急停止检查** / Emergency Stop Check: 检查手柄急停状态
2. **速度命令处理** / Velocity Command Processing: 获取用户输入的速度命令
3. **步态调度更新** / Gait Scheduling Update: 根据当前步态周期生成步态调度
4. **状态估计更新** / State Estimation Update: 融合传感器数据估计机器人状态
5. **轨迹生成** / Trajectory Generation: MPC求解最优接触力和状态轨迹
6. **控制器执行** / Controller Execution: 全身控制器将MPC结果转换为关节控制命令
7. **数据可视化** / Data Visualization: 更新可视化显示

### 详细控制流程 / Detailed Control Flow
1. **状态估计** / State Estimation: 融合传感器数据估计机器人状态
2. **步态规划** / Gait Planning: 根据用户命令规划步态序列
3. **MPC优化** / MPC Optimization: 求解最优接触力和状态轨迹
4. **全身控制** / Whole Body Control: 将MPC结果转换为关节控制命令
5. **执行器控制** / Actuator Control: 发送控制命令到机器人关节

## 关键算法说明 / Key Algorithm Explanations

### 凸优化MPC / Convex MPC
- 将非线性双足动力学线性化
- 使用HPIPM求解器求解二次规划问题
- 考虑接触约束和摩擦锥约束

### 全身控制WBC / Whole Body Control
- 基于任务空间控制框架
- 使用二次规划处理多任务和约束
- 包含动力学一致性约束

### 状态估计 / State Estimation
- 线性卡尔曼滤波融合多传感器数据
- 处理足端接触检测和运动估计

## 修改的文件列表 / Modified File List

1. `src/motion/generation/include/generation/ConvexMPC.h`
2. `src/motion/generation/src/ConvexMPC.cc`
3. `src/motion/control/include/control/WholeBodyController.h`
4. `src/motion/control/src/WholeBodyController.cc`
5. `src/motion/estimation/include/estimation/StateEstimationLKF.h`
6. `src/motion/management/include/MotionManager.h`
7. `src/motion/management/src/MotionManager.cc` ⭐ **新增注释**
8. `src/core/include/core/gait/MotionPhaseDefinition.h`
9. `src/core/include/core/gait/LegLogic.h`
10. `src/core/include/core/types.h`
11. `README.md`
12. `CODE_DOCUMENTATION_SUMMARY.md`

## 注释风格 / Comment Style

所有注释采用中英文对照格式：
All comments use bilingual Chinese-English format:

```cpp
/**
 * @brief 中文描述 / English description
 * @param param_name 参数描述 / Parameter description
 * @return 返回值描述 / Return value description
 * 
 * 详细中文说明
 * Detailed English explanation
 */
```

