// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once
#include "resim/utils/mcap_logger.hh"

namespace resim::simulator {

// A very simple interface used for independent units in our simulator
// that set up tasks and own the data they read and write throughout
// the course of a simulation.
class SimulationUnit {
 public:
  explicit SimulationUnit(std::shared_ptr<LoggerInterface> logger_interface)
      : logger_(std::move(logger_interface)){};
  SimulationUnit(const SimulationUnit &) = delete;
  SimulationUnit(SimulationUnit &&) noexcept = delete;
  SimulationUnit &operator=(SimulationUnit &&) noexcept = delete;
  SimulationUnit &operator=(const SimulationUnit &) = delete;
  virtual ~SimulationUnit() = default;

 protected:
  std::shared_ptr<LoggerInterface> logger() const { return logger_; };

 private:
  std::shared_ptr<LoggerInterface> logger_;
};

}  // namespace resim::simulator
