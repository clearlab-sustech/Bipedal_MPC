#include "core/gait/ModeSchedule.h"
#include "core/misc/NumericTraits.h"
#include <algorithm>
#include <cmath>
#include <core/misc/Numerics.h>
#include <iostream>
#include <rcpputils/asserts.hpp>

namespace clear {

ModeSchedule::ModeSchedule(scalar_t duration,
                           std::vector<scalar_t> &eventPhases,
                           std::vector<size_t> &modeSequence,
                           scalar_t gait_cycle)
    : duration_(duration), eventPhases_(std::move(eventPhases)),
      modeSequence_(std::move(modeSequence)), gait_cycle_(gait_cycle) {}

scalar_t ModeSchedule::duration() const { return duration_; }

scalar_t ModeSchedule::gaitCycle() const { return gait_cycle_; }

const std::vector<scalar_t> &ModeSchedule::eventPhases() const {
  return eventPhases_;
}

const std::vector<size_t> &ModeSchedule::modeSequence() const {
  return modeSequence_;
}

bool ModeSchedule::isValidModeSequence() const {
  bool validGait = true;
  validGait &= duration_ > 0.0;
  validGait &= numerics::almost_eq(eventPhases_.front(), 0.0);
  validGait &= numerics::almost_eq(eventPhases_.back(), 1.0);
  validGait &=
      std::all_of(eventPhases_.begin(), eventPhases_.end(), [](scalar_t phase) {
        return -numeric_traits::limitEpsilon<scalar_t>() < phase &&
               phase < 1.0 + numeric_traits::limitEpsilon<scalar_t>();
      });
  validGait &= std::is_sorted(eventPhases_.begin(), eventPhases_.end());
  validGait &= eventPhases_.size() - 1 == modeSequence_.size();
  return validGait;
}

scalar_t ModeSchedule::getEventPhaseFromModeIndex(const size_t mode_idx) const {
  scalar_t event_phase = 0.0;
  for (size_t k = 0; k < modeSequence_.size(); k++) {
    if (modeSequence_[k] == mode_idx) {
      event_phase = eventPhases_[k];
    }
  }
  return event_phase;
}

bool ModeSchedule::isValidPhase(scalar_t phase) const {
  return phase >= 0.0 && phase <= 1.0;
}

scalar_t ModeSchedule::wrapPhase(scalar_t phase) const {
  phase = std::fmod(phase, 1.0);
  if (phase < 0.0) {
    phase += 1.0;
  }
  return phase;
}

int ModeSchedule::getModeIndexFromPhase(scalar_t phase) const {
  rcpputils::assert_true(isValidPhase(phase));
  rcpputils::assert_true(isValidModeSequence());
  auto firstLargerValueIterator =
      std::upper_bound(eventPhases_.begin(), eventPhases_.end(), phase);
  if (firstLargerValueIterator != eventPhases_.end()) {
    return std::max(
        static_cast<int>(firstLargerValueIterator - eventPhases_.begin()) - 1,
        0);
  } else {
    return modeSequence_.size() - 1;
  }
}

size_t ModeSchedule::getModeFromPhase(scalar_t phase) const {
  rcpputils::assert_true(isValidPhase(phase));
  rcpputils::assert_true(isValidModeSequence());
  return modeSequence_[getModeIndexFromPhase(phase)];
}

scalar_t ModeSchedule::timeLeftInModeSequence(scalar_t phase) const {
  rcpputils::assert_true(isValidPhase(phase));
  rcpputils::assert_true(isValidModeSequence());
  return (1.0 - phase) * duration_;
}

scalar_t ModeSchedule::timeLeftInMode(scalar_t phase) const {
  rcpputils::assert_true(isValidPhase(phase));
  rcpputils::assert_true(isValidModeSequence());
  int modeIndex = getModeIndexFromPhase(phase);
  if (static_cast<size_t>(modeIndex) < eventPhases_.size()) {
    return (eventPhases_[modeIndex + 1] - phase) * duration_;
  } else {
    return timeLeftInModeSequence(phase);
  }
}

void ModeSchedule::print() const {
  std::cout << "#### Duration:       " << duration_ << "\n";
  std::cout << "#### Event phases:  {";
  for (auto &phase : eventPhases_) {
    std::cout << phase << ", ";
  }
  std::cout << "}\n";
  std::cout << "#### Mode sequence: {";
  for (auto &mode : modeSequence_) {
    std::cout << mode << ", ";
  }
  std::cout << "}\n";
}
} // namespace clear
