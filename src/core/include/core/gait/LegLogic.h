#pragma once

#include <core/gait/ModeSchedule.h>
#include <core/gait/MotionPhaseDefinition.h>

#include <memory>
#include <vector>

namespace clear {
namespace biped {

/**
 * @brief 时间区间结构体 / Time Interval Structure
 * 
 * 表示一个时间段，包含开始时间和结束时间
 * Represents a time period with start and end times
 */
struct TimeInterval {
  scalar_t start;  // 开始时间 / Start time
  scalar_t end;    // 结束时间 / End time
};

/**
 * @brief 获取下次着地时间 / Get time of next touchdown
 * @param time_cur 当前时间 / Current time
 * @param mode_schedule 步态调度器 / Mode scheduler
 * @return 每只脚的下次着地时间向量 / Vector of next touchdown times for each foot
 * 
 * 根据当前步态调度计算每只脚的下次着地时间，用于步态规划和预测
 * Calculate next touchdown time for each foot based on current gait schedule,
 * used for gait planning and prediction
 */
std::vector<scalar_t>
getTimeOfNextTouchDown(scalar_t time_cur,
                       const std::shared_ptr<ModeSchedule> mode_schedule);

/**
 * @brief 获取下次抬起时间 / Get time of next lift off
 * @param time_cur 当前时间 / Current time
 * @param mode_schedule 步态调度器 / Mode scheduler
 * @return 每只脚的下次抬起时间向量 / Vector of next lift off times for each foot
 * 
 * 根据当前步态调度计算每只脚的下次抬起时间，用于摆动腿轨迹规划
 * Calculate next lift off time for each foot based on current gait schedule,
 * used for swing leg trajectory planning
 */
std::vector<scalar_t>
getTimeOfNextLiftOff(scalar_t time_cur,
                     const std::shared_ptr<ModeSchedule> mode_schedule);

} // namespace biped
} // namespace clear
