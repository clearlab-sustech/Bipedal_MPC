#include "gait/GaitSchedule.h"
#include "core/misc/Numerics.h"

namespace clear {

GaitSchedule::GaitSchedule(Node::SharedPtr nodeHandle)
    : nodeHandle_(nodeHandle) {

  const std::string config_file_ = nodeHandle_->get_parameter("/config_file")
                                       .get_parameter_value()
                                       .get<std::string>();

  auto config_ = YAML::LoadFile(config_file_);
  gait_list = config_["gait"]["list"].as<std::vector<std::string>>();

  gait_map_.clear();
  for (const auto &gaitName : gait_list) {
    gait_map_.insert({gaitName, loadGait(config_, gaitName)});
  }

  current_gait_.push(gait_list[1]);
  gait_buffer_.push(gait_list[1]);
  transition_time_.push(nodeHandle_->now().seconds());

  check_.push(true);
  in_transition_.push(false);

  std::string topic_prefix = config_["global"]["topic_prefix"].as<std::string>();
  robot_name = config_["model"]["name"].as<std::string>();
  gait_service_ = nodeHandle_->create_service<trans::srv::GaitSwitch>(
      topic_prefix + "gait_switch",
      std::bind(&GaitSchedule::gaitSwitch, this, std::placeholders::_1,
                std::placeholders::_2));

  cycle_timer_ =
      std::make_shared<CycleTimer>(nodeHandle_->shared_from_this(),
                                   gait_map_[current_gait_.get()]->duration());
}

GaitSchedule::~GaitSchedule() {
  check_.push(false);
  check_transition_thread_.join();
}

std::shared_ptr<ModeSchedule>
GaitSchedule::loadGait(const YAML::Node node, const std::string &gait_name) {
  scalar_t duration;
  std::vector<scalar_t> eventPhases =
      node["gait"][gait_name]["switchingTimes"].as<std::vector<scalar_t>>();

  scalar_t end_phase = eventPhases.back();
  duration = end_phase;
  if (!numerics::almost_eq(end_phase, 1.0) &&
      end_phase > numeric_traits::limitEpsilon<scalar_t>()) {
    for (auto &phase : eventPhases) {
      phase /= end_phase;
    }
  }

  std::vector<size_t> modeSequence;
  std::vector<std::string> modeSequenceString =
      node["gait"][gait_name]["modeSequence"].as<std::vector<std::string>>();

  if (eventPhases.empty() || modeSequenceString.empty()) {
    throw std::runtime_error("[loadModeSequenceTemplate] failed to load : " +
                             gait_name);
  }
  // convert the mode name to mode enum
  modeSequence.reserve(modeSequenceString.size());
  for (const auto &modeName : modeSequenceString) {
    modeSequence.push_back(biped::string2ModeNumber(modeName));
  }
  auto gait =
      std::make_shared<ModeSchedule>(duration, eventPhases, modeSequence);
  if (!gait->isValidModeSequence()) {
    throw std::runtime_error(gait_name + " gait is not valid");
  }
  return gait;
}

void GaitSchedule::gaitSwitch(
    const std::shared_ptr<trans::srv::GaitSwitch::Request> request,
    std::shared_ptr<trans::srv::GaitSwitch::Response> response) {
  if (request->header.frame_id != robot_name) {
    RCLCPP_WARN(
        rclcpp::get_logger("GaitSchedule"),
        "gait switch request id: %s, and the robot id: %s, not compatible",
        request->header.frame_id.c_str(), robot_name.c_str());
    response->is_success = false;
  } else {
    if (std::find(gait_list.begin(), gait_list.end(), request->gait_name) !=
        gait_list.end()) {
      RCLCPP_WARN(rclcpp::get_logger("GaitSchedule"), "gait request: change to %s",
                  request->gait_name.c_str());
      if (request->gait_name == current_gait_.get()) {
        RCLCPP_WARN(rclcpp::get_logger("GaitSchedule"), "It has been in %s ",
                    current_gait_.get().c_str());
      } else if (in_transition_.get()) {
        RCLCPP_WARN(rclcpp::get_logger("GaitSchedule"),
                    "It is in %s transition process,  try call gait switch "
                    "service latter",
                    gait_buffer_.get().c_str());
      } else {
        if (check_transition_thread_.joinable())
          check_transition_thread_.join();

        auto current_gait_name = current_gait_.get();
        scalar_t phase_ = cycle_timer_->getCycleTime() /
                          gait_map_[current_gait_name]->duration();
        transition_time_.push(
            nodeHandle_->now().seconds() +
            gait_map_[current_gait_name]->timeLeftInModeSequence(phase_));
        gait_buffer_.push(request->gait_name);
        check_transition_thread_ =
            std::thread(&GaitSchedule::checkGaitTransition, this);
        RCLCPP_WARN(rclcpp::get_logger("GaitSchedule"), "gait %s will start at time %f",
                    gait_buffer_.get().c_str(), transition_time_.get());
      }

      response->is_success = true;
    } else {
      RCLCPP_WARN(rclcpp::get_logger("GaitSchedule"), "not found gait %s in gait list",
                  request->gait_name.c_str());
      response->is_success = false;
    }
  }
}

void GaitSchedule::switchGait(std::string gait_name) {
  if (std::find(gait_list.begin(), gait_list.end(), gait_name) !=
      gait_list.end()) {
    if (gait_name != current_gait_.get() && !in_transition_.get()) {
      RCLCPP_WARN(rclcpp::get_logger("GaitSchedule"), "gait request: change to %s",
                  gait_name.c_str());
      if (check_transition_thread_.joinable())
        check_transition_thread_.join();

      auto current_gait_name = current_gait_.get();
      scalar_t phase_ = cycle_timer_->getCycleTime() /
                        gait_map_[current_gait_name]->duration();
      if (current_gait_name == "stance") {
        transition_time_.push(nodeHandle_->now().seconds() + 0.05);
      } else if (gait_name == "stance") {
        transition_time_.push(
            nodeHandle_->now().seconds() +
            gait_map_[current_gait_name]->timeLeftInModeSequence(phase_) +
            gait_map_[current_gait_name]->duration());
      } else {
        transition_time_.push(
            nodeHandle_->now().seconds() +
            gait_map_[current_gait_name]->timeLeftInModeSequence(phase_));
      }
      gait_buffer_.push(gait_name);
      check_transition_thread_ =
          std::thread(&GaitSchedule::checkGaitTransition, this);
      RCLCPP_WARN(rclcpp::get_logger("GaitSchedule"), "gait %s will start at time %f",
                  gait_buffer_.get().c_str(), transition_time_.get());
    }
  }
}

size_t GaitSchedule::currentMode() {
  auto current_gait_name = current_gait_.get();
  scalar_t phase_ =
      cycle_timer_->getCycleTime() / gait_map_[current_gait_name]->duration();
  return gait_map_[current_gait_name]->getModeFromPhase(phase_);
}

scalar_t GaitSchedule::currentGaitCycle() {
  return gait_map_[current_gait_.get()]->duration();
}

std::shared_ptr<ModeSchedule> GaitSchedule::eval(scalar_t time_period) {
  scalar_t duration;
  std::vector<scalar_t> eventPhases;
  std::vector<size_t> modeSequence;

  auto current_gait_name = current_gait_.get();
  duration = time_period;
  auto &gait_cur = gait_map_[current_gait_name];
  const auto &gait_duration = gait_map_[current_gait_name]->duration();
  const auto &gait_seq = gait_map_[current_gait_name]->modeSequence();
  const auto &gait_event_phase = gait_map_[current_gait_name]->eventPhases();

  scalar_t time_c = cycle_timer_->getCycleTime();
  scalar_t phase_c = time_c / gait_duration;
  int idx = gait_cur->getModeIndexFromPhase(phase_c);

  eventPhases.push_back(0.0);
  for (size_t i = static_cast<size_t>(idx); i < gait_seq.size(); i++) {
    modeSequence.push_back(gait_seq[i]);
    scalar_t phase_i =
        (gait_event_phase[i + 1] * gait_duration - time_c) / time_period;
    if (phase_i < 1.0) {
      eventPhases.push_back(phase_i);
    } else {
      eventPhases.push_back(1.0);
      break;
    }
  }

  scalar_t time_l = time_period - (gait_duration - time_c);
  scalar_t cycle_n = 1.0;
  while (time_l > 0) {
    for (size_t i = 0; i < gait_seq.size(); i++) {
      modeSequence.push_back(gait_seq[i]);
      scalar_t phase_i =
          ((gait_event_phase[i + 1] + cycle_n) * gait_duration - time_c) /
          time_period;
      if (phase_i < 1.0) {
        eventPhases.push_back(phase_i);
      } else {
        eventPhases.push_back(1.0);
        break;
      }
    }
    cycle_n += 1.0;
    time_l -= gait_duration;
  }
  auto modeSchedule =
      std::make_shared<ModeSchedule>(duration, eventPhases, modeSequence, gait_duration);
  if (!modeSchedule->isValidModeSequence()) {
    throw std::runtime_error(
        "GaitSchedule::eval >>> modeSchedule is not valid");
  }
  return modeSchedule;
}

void GaitSchedule::checkGaitTransition() {
  in_transition_.push(true);

  if (transition_time_.get() < nodeHandle_->now().seconds() ||
      transition_time_.get() - nodeHandle_->now().seconds() > 1e2) {
    printf("transition time slot: %fs",
           transition_time_.get() - nodeHandle_->now().seconds());
    throw std::runtime_error("transition time slot is not valid [0, 100]");
  }

  rclcpp::Rate loop_rate(1000.0);
  while (rclcpp::ok() && check_.get() &&
         gait_buffer_.get() != current_gait_.get()) {
    if (nodeHandle_->now().seconds() > transition_time_.get()) {
      current_gait_.push(gait_buffer_.get());
      cycle_timer_ = std::make_shared<CycleTimer>(
          nodeHandle_->shared_from_this(),
          gait_map_[current_gait_.get()]->duration());
    }
    loop_rate.sleep();
  }
  RCLCPP_WARN(rclcpp::get_logger("GaitSchedule"), "gait has been changed to %s",
              current_gait_.get().c_str());

  in_transition_.push(false);
}
std::string GaitSchedule::getCurrentGaitName() { return current_gait_.get(); }

} // namespace clear
