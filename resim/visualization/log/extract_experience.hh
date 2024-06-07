// Copyright 2024 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <mcap/reader.hpp>

#include "resim/experiences/experience.hh"
#include "resim/utils/inout.hh"

namespace resim::visualization::log {

// This function extracts the experience config from the given log (represented
// by a log reader). It expects one and exactly one such message to be logged
// on the "/experience" topic and throws otherwise.
// @param[in] reader - The reader for the log we want to get the experience
//                     config from.
// @returns The experience extracted from this log.
// @throws AssertException if zero or more than one experiences are logged.
experiences::Experience extract_experience(InOut<mcap::McapReader> reader);

}  // namespace resim::visualization::log
