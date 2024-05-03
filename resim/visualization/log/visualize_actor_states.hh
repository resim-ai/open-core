
#pragma once

#include <mcap/reader.hpp>

#include "resim/experiences/experience.hh"
#include "resim/utils/inout.hh"
#include "resim/utils/mcap_logger.hh"

namespace resim::visualization::log {

// This function is responsible for reading the actor states from the given
// input log (reader) and creating appropriate ::foxglove::FrameTransform and
// ::foxglove::SceneUpdates on the "/transforms/scene_from_<actor_frame_id>/"
// topics for the former and the "/geometries" topic for the latter. More
// simply, this function puts content in the mcap such that Foxglove studio
// will display frames for each actor and actor geometries (where specified) in
// those frames.
// @param[in] experience - The experience to get the geometries from.
// @param[in] reader - The log to read actors from.
// @param[out] logger - The logger to log ::foxglove::FrameTransforms and
//                      ::foxglove::SceneUpdates too.
void visualize_actor_states(
    const experiences::Experience &experience,
    InOut<mcap::McapReader> reader,
    InOut<McapLogger> logger);

}  // namespace resim::visualization::log
