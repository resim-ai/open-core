// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.


#include "resim/simulator/time_lord.hh"

#include "resim/assert/assert.hh"
#include "resim/simulator/simulation_unit.hh"
#include "resim/simulator/standard_topics.hh"

namespace resim::simulator {

TimeLord::TimeLord(
    std::shared_ptr<LoggerInterface> logger_interface,
    InOut<ExecutorBuilder> executor_builder)
    : SimulationUnit(std::move(logger_interface)) {
  executor_builder->add_independent_task<time::Timestamp>(
      "publish_time",
      TIME_TOPIC,
      [this]() {
        REASSERT(!ready_to_terminate());

        TimeLordUpdate update{scheduled_updates_.top()};
        scheduled_updates_.pop();
        if (current_time_.has_value()) {
          REASSERT(update.time > current_time_.value());
        }
        current_time_ = update.time;

        // Pop any repeats of the same time
        while (not scheduled_updates_.empty() &&
               scheduled_updates_.top().time <= current_time_.value()) {
          REASSERT(
              scheduled_updates_.top().time == current_time_.value(),
              "This ordering of timings should never happen.");
          scheduled_updates_.pop();
        }

        if (update.update_behaviour ==
            TimeLordUpdate::FULL_UPDATE_THEN_TERMINATE) {
          terminate_flag_ = true;
        }

        return current_time_.value();
      });
  executor_builder->add_task<std::vector<TimeLordUpdate>>(
      "schedule_timelord_updates",
      SCHEDULE_TIMELORD_TOPIC,
      NULL_TOPIC,
      [this](const std::vector<std::vector<TimeLordUpdate>>& updates_list) {
        for (const std::vector<TimeLordUpdate>& updates : updates_list) {
          for (const TimeLordUpdate& update : updates) {
            // These types of updates are those scheduled by another unit,
            // as opposed to prescheduled updates.
            REASSERT(
                update.update_type ==
                TimeLordUpdate::UpdateType::UNIT_SCHEDULED);
            REASSERT(
                update.update_behaviour !=
                TimeLordUpdate::UpdateBehaviour::UNSUPPORTED);

            if (update.time == current_time_) {
              if (update.update_behaviour ==
                  TimeLordUpdate::FULL_UPDATE_THEN_TERMINATE) {
                terminate_flag_ = true;
              }
            } else {
              this->schedule_update(update);
            }
          }
        }
      });
}

// Returns whether the time lord is ready to terminate,
// which means it's been signalled to terminate, or
// has run out of updates.
// This is currently how sim termination is handled.
bool TimeLord::ready_to_terminate() {
  return terminate_flag_ || scheduled_updates_.empty();
}

// Schedules a sim update to be run at a certain timestamp. At present,
// only full updates (where all units run) are supported. The accessor to this
// method for other units is writing updates to the SCHEDULE_TIMELORD_TOPIC, the
// interface of which can be seen above.
void TimeLord::schedule_update(const TimeLordUpdate& update) {
  REASSERT(
      (not current_time_.has_value() || update.time > current_time_),
      "TimeLord update scheduled at or before current time");
  REASSERT(
      update.update_behaviour != TimeLordUpdate::UpdateBehaviour::UNSUPPORTED,
      "Unsupported behaviour");
  scheduled_updates_.emplace(update);
}

}  // namespace resim::simulator
