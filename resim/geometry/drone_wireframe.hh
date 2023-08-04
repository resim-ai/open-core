// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

//
//  drone_wireframe.hh
//
// This library contains a function to produce a very simple wireframe for a
// quad-copter drone. It creates an "X" shaped chassis with an additional
// cross-beam crossing the negative x axis (so the orientation is easily
// discernable). On the end of each leg of the chassis, there is an upward-bent
// member which is protrudes by a given lateral and vertical offset to the point
// where a rotor is mounted for that leg. The rotor is represented by a
// horizontal circle centered atthe end of this upward-bent member. This library
// is essentially a replacement for configuration until we select a format for
// that.
//
#pragma once

#include "resim/geometry/wireframe.hh"

namespace resim::geometry {

struct DroneExtents {
  double chassis_radius_m = 0.;
  double rotor_lateral_offset_m = 0.;
  double rotor_vertical_offset_m = 0.;
  double rotor_radius_m = 0.;
  std::size_t samples_per_rotor = 0;
};

// Create a simple wireframe for a drone.
// @param[in] drone_extents - The extents to use for the drone wireframe.
// @throws If the chassis or rotor radii are non-positive or the number of
// samples per rotor is less than 2.
Wireframe drone_wireframe(const DroneExtents &drone_extents);

}  // namespace resim::geometry
