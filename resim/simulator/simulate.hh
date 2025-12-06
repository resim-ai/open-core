// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <filesystem>
#include <memory>

#include "resim/experiences/experience.hh"

namespace resim::simulator {

// This function runs a simulation based on an experience config and saves the
// result to the given mcap path.
// @param[in] experience - The experience config for this simulation.
// @param[in] mcap_path - The path of the MCAP file to log the simulation
// results to.
void simulate(
    experiences::Experience &experience,
    const std::filesystem::path &mcap_path);

}  // namespace resim::simulator
