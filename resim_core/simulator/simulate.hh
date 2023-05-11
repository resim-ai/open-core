#pragma once

#include <filesystem>
#include <memory>

#include "resim_core/experiences/experience.hh"

namespace resim::simulator {

// This function runs a simulation based on an experience config and saves the
// result to the given mcap path.
// @param[in] experience - The experience config for this simulation.
// @param[in] mcap_path - The path of the MCAP file to log the simulation
// results to.
void simulate(
    const experiences::Experience &experience,
    const std::filesystem::path &mcap_path);

}  // namespace resim::simulator
