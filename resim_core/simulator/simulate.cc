
#include "resim_core/simulator/simulate.hh"

#include <chrono>
#include <memory>
#include <utility>
#include <vector>

#include "resim_core/actor/actor_unit.hh"
#include "resim_core/actor/factory.hh"
#include "resim_core/metrics/actor_metrics_unit.hh"
#include "resim_core/simulator/executor_builder.hh"
#include "resim_core/simulator/step_executor.hh"
#include "resim_core/simulator/time_lord.hh"
#include "resim_core/time/timestamp.hh"
#include "resim_core/utils/inout.hh"
#include "resim_core/utils/mcap_logger.hh"

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

  for (time::Timestamp t; t <= time_limit; t += DT) {
    executor->run_step();
    time_lord->increment_time(DT);
  }
}

}  // namespace resim::simulator
