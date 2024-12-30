#pragma once

#include "core/types.h"
#include <ostream>
#include <vector>

namespace clear {

class ModeSchedule {
private:
  scalar_t duration_;
  std::vector<scalar_t> eventPhases_;
  std::vector<size_t> modeSequence_;
  scalar_t gait_cycle_;

public:
  ModeSchedule(scalar_t duration, std::vector<scalar_t> &eventPhases,
               std::vector<size_t> &modeSequence, scalar_t gait_cycle = 1.0);

  ~ModeSchedule() = default;

  scalar_t duration() const;

  scalar_t gaitCycle() const;

  const std::vector<scalar_t> &eventPhases() const;

  const std::vector<size_t> &modeSequence() const;

  /**
   * isValidModeSequence checks the following properties
   * - positive duration
   * - eventPhases are all in (0.0, 1.0)
   * - eventPhases are sorted
   * - the size of the modeSequences is 1 less than the eventPhases.
   */
  bool isValidModeSequence() const;

  /** The event phase of the mode */
  scalar_t getEventPhaseFromModeIndex(const size_t mode_idx) const;

  /** Check is if the phase is in [0.0, 1.0) */
  bool isValidPhase(scalar_t phase) const;

  /** Wraps a phase to [0.0, 1.0) */
  scalar_t wrapPhase(scalar_t phase) const;

  /** The modes are selected with a closed-open interval: [ ) */
  int getModeIndexFromPhase(scalar_t phase) const;

  /** Gets the active mode from the phase variable */
  size_t getModeFromPhase(scalar_t phase) const;

  /** Returns the time left in the modeSequence based on the phase variable */
  scalar_t timeLeftInModeSequence(scalar_t phase) const;

  /** Returns the time left in the current mode sequence based on the phase
   * variable */
  scalar_t timeLeftInMode(scalar_t phase) const;

  /** Print modeSequence */
  void print() const;
};

} // namespace clear
