#pragma once

#include "core/types.h"
#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <iostream>
#include <map>
#include <string>
#include <vector>

namespace clear {

namespace biped {
/**
 * @brief 双足机器人类型定义 / Bipedal robot type definitions
 */
template <typename T> using feet_array_t = std::array<T, 2>;  // 双足数组模板 / Two-foot array template
using contact_flag_t = feet_array_t<bool>;                    // 接触标志类型 / Contact flag type

using vector3_t = Eigen::Matrix<scalar_t, 3, 1>;             // 3D向量类型 / 3D vector type
using matrix3_t = Eigen::Matrix<scalar_t, 3, 3>;             // 3x3矩阵类型 / 3x3 matrix type
using quaternion_t = Eigen::Quaternion<scalar_t>;            // 四元数类型 / Quaternion type

/**
 * @brief 步态模式枚举 / Gait mode enumeration
 * 
 * 定义双足机器人的不同接触状态
 * Defines different contact states for bipedal robot
 * {左足, 右足} / {Left Foot, Right Foot}
 */
enum ModeNumber {
  FLY = 0,      // 飞行期：双足离地 / Flight phase: both feet off ground
  LF = 1,       // 左足支撑期：左足接触，右足摆动 / Left foot stance: left foot contact, right foot swing
  RF = 2,       // 右足支撑期：右足接触，左足摆动 / Right foot stance: right foot contact, left foot swing
  STANCE = 3,   // 双足支撑期：双足接触 / Double stance: both feet contact
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
/**
 * @brief 将模式编号转换为支撑腿接触标志 / Convert mode number to stance leg contact flags
 * @param modeNumber 步态模式编号 / Gait mode number
 * @return 接触标志数组 {左足接触, 右足接触} / Contact flag array {left foot contact, right foot contact}
 * 
 * 将步态模式编号映射到具体的足端接触状态
 * Maps gait mode numbers to specific foot contact states
 */
inline contact_flag_t modeNumber2StanceLeg(const size_t &modeNumber) {
  contact_flag_t stanceLegs; // {左足, 右足} / {Left Foot, Right Foot}

  switch (modeNumber) {
  case 0:
    stanceLegs = contact_flag_t{false, false};  // 飞行期：双足离地 / Flight: both feet off ground
    break;
  case 1:
    stanceLegs = contact_flag_t{true, false};   // 左足支撑 / Left foot stance
    break;
  case 2:
    stanceLegs = contact_flag_t{false, true};   // 右足支撑 / Right foot stance
    break;
  case 3:
    stanceLegs = contact_flag_t{true, true};    // 双足支撑 / Double stance
    break;
  }

  return stanceLegs;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline size_t stanceLeg2ModeNumber(const contact_flag_t &stanceLegs) {
  return static_cast<size_t>(stanceLegs[0]) +
         2 * static_cast<size_t>(stanceLegs[2]);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline std::string modeNumber2String(const size_t &modeNumber) {
  // build the map from mode number to name
  std::map<size_t, std::string> modeToName;
  modeToName[FLY] = "FLY";
  modeToName[RF] = "RF";
  modeToName[LF] = "LF";
  modeToName[STANCE] = "STANCE";
  return modeToName[modeNumber];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline size_t string2ModeNumber(const std::string &modeString) {
  // build the map from name to mode number
  std::map<std::string, size_t> nameToMode;
  nameToMode["FLY"] = FLY;
  nameToMode["RF"] = RF;
  nameToMode["LF"] = LF;
  nameToMode["STANCE"] = STANCE;
  return nameToMode[modeString];
}

} // namespace biped

} // end of namespace clear
