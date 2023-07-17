
#include "resim/simulator/simulate.hh"

#include <chrono>
#include <memory>
#include <utility>
#include <vector>

#include "resim/actor/actor_unit.hh"
#include "resim/actor/factory.hh"
#include "resim/experiences/proto/experience.pb.h"
#include "resim/experiences/proto/experience_to_proto.hh"
#include "resim/metrics/actor_metrics_unit.hh"
#include "resim/simulator/executor_builder.hh"
#include "resim/simulator/step_executor.hh"
#include "resim/simulator/time_lord.hh"
#include "resim/time/timestamp.hh"
#include "resim/utils/inout.hh"
#include "resim/utils/mcap_logger.hh"

namespace resim::simulator {

void simulate(
    const experiences::Experience &experience,
    const std::filesystem::path &mcap_path) {
  const time::Timestamp time_limit{
      experience.dynamic_behavior.completion_criteria.time_limit};

  REASSERT(time_limit > time::Timestamp(), "Time limit must be positive!");

  // Setup logger
  std::shared_ptr<LoggerInterface> logger{
      std::make_shared<McapLogger>(mcap_path)};

  // Log the experience
  experiences::proto::Experience experience_msg;
  pack(experience, &experience_msg);
  constexpr time::Timestamp ZERO;
  logger->add_proto_channel<experiences::proto::Experience>("/experience");
  logger->log_proto("/experience", ZERO, experience_msg);

  // Setup units
  ExecutorBuilder executor_builder;

  auto time_lord = std::make_unique<TimeLord>(logger, InOut{executor_builder});

  auto actor_logger_unit =
      std::make_unique<actor::ActorLoggerUnit>(logger, InOut{executor_builder});

  std::vector<std::unique_ptr<actor::ActorUnit>> actor_units;
  std::vector<std::unique_ptr<actor::Actor>> actors{
      actor::factory(experience.dynamic_behavior, experience.geometries)};

  // TODO(tknowles): For now, add an actor metrics unit for the first actor
  if (not actors.empty()) {
    metrics::ActorMetricsUnit ego_metrics_unit{
        logger,
        InOut{executor_builder},
        actors.front()->id()};
  }

  actor_units.reserve(actors.size());
  for (auto &actor : actors) {
    actor_units.push_back(std::make_unique<actor::ActorUnit>(
        logger,
        std::move(actor),
        InOut{executor_builder}));
  }
  std::unique_ptr<StepExecutor> executor = executor_builder.build();

  // Run the sim
  constexpr time::Duration DT{std::chrono::milliseconds(10)};

  time::Timestamp t;
  for (; t < time_limit; t += DT) {
    time_lord->schedule_update(TimeLordUpdate(
        t,
        TimeLordUpdate::FULL_UPDATE,
        TimeLordUpdate::PRESCHEDULED));
  }

  time_lord->schedule_update(TimeLordUpdate(
      time_limit,
      TimeLordUpdate::FULL_UPDATE_THEN_TERMINATE,
      TimeLordUpdate::PRESCHEDULED));

  while (not time_lord->ready_to_terminate()) {
    executor->run_step();
  }
}

}  // namespace resim::simulator
