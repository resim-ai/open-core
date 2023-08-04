// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/geometry/drone_wireframe.hh"

#include <gtest/gtest.h>

#include "resim/assert/assert.hh"

namespace resim::geometry {

// This library is essentially a replacement for configuration until we choose a
// format in which to store it, so for now, we just test that it produces a
// valid wireframe.
TEST(DroneWireframeTest, TestDroneWireframe) {
  // SETUP
  constexpr double CHASSIS_RADIUS_M = 1.;
  constexpr double ROTOR_LATERAL_OFFSET_M = 0.3;
  constexpr double ROTOR_VERTICAL_OFFSET_M = 0.3;
  constexpr double ROTOR_RADIUS_M = 0.5;
  constexpr std::size_t SAMPLES_PER_ROTOR = 2;

  const DroneExtents extents{
      .chassis_radius_m = CHASSIS_RADIUS_M,
      .rotor_lateral_offset_m = ROTOR_LATERAL_OFFSET_M,
      .rotor_vertical_offset_m = ROTOR_VERTICAL_OFFSET_M,
      .rotor_radius_m = ROTOR_RADIUS_M,
      .samples_per_rotor = SAMPLES_PER_ROTOR,
  };

  // ACTION
  const Wireframe drone{drone_wireframe(extents)};

  // VERIFICATION
  EXPECT_TRUE(drone.is_valid());
}

TEST(DroneWireframeDeathTest, TestFailsOnInvalidExtents) {
  // SETUP
  constexpr double CHASSIS_RADIUS_M = 1.;
  constexpr double ROTOR_LATERAL_OFFSET_M = 0.3;
  constexpr double ROTOR_VERTICAL_OFFSET_M = 0.3;
  constexpr double ROTOR_RADIUS_M = 0.5;
  constexpr std::size_t SAMPLES_PER_ROTOR = 2;

  const DroneExtents base_extents{
      .chassis_radius_m = CHASSIS_RADIUS_M,
      .rotor_lateral_offset_m = ROTOR_LATERAL_OFFSET_M,
      .rotor_vertical_offset_m = ROTOR_VERTICAL_OFFSET_M,
      .rotor_radius_m = ROTOR_RADIUS_M,
      .samples_per_rotor = SAMPLES_PER_ROTOR,
  };

  DroneExtents invalid_chassis_radius{base_extents};
  invalid_chassis_radius.chassis_radius_m = -1.0;

  DroneExtents invalid_rotor_radius{base_extents};
  invalid_rotor_radius.rotor_radius_m = -1.0;

  DroneExtents invalid_samples_per_rotor{base_extents};
  invalid_samples_per_rotor.samples_per_rotor = 1;

  // ACTION / VERIFICATION
  EXPECT_THROW(drone_wireframe(invalid_chassis_radius), AssertException);
  EXPECT_THROW(drone_wireframe(invalid_rotor_radius), AssertException);
  EXPECT_THROW(drone_wireframe(invalid_samples_per_rotor), AssertException);
}

}  // namespace resim::geometry
