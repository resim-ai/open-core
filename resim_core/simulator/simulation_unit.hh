#pragma once

namespace resim::simulator {

// A very simple interface used for independent units in our simulator
// that set up tasks and own the data they read and write throughout
// the course of a simulation.
class SimulationUnit {
 public:
  SimulationUnit() = default;
  SimulationUnit(const SimulationUnit &) = default;
  SimulationUnit(SimulationUnit &&) noexcept = default;
  SimulationUnit &operator=(SimulationUnit &&) noexcept = default;
  SimulationUnit &operator=(const SimulationUnit &) = default;
  virtual ~SimulationUnit() = default;
};

}  // namespace resim::simulator
