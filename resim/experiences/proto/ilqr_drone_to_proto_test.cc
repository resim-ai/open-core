// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/experiences/proto/ilqr_drone_to_proto.hh"

#include <gtest/gtest.h>

#include "resim/assert/assert.hh"
#include "resim/experiences/ilqr_drone.hh"
#include "resim/experiences/proto/experiences_test_helpers.hh"
#include "resim/experiences/proto/ilqr_drone.pb.h"

namespace resim::experiences {

TEST(ILQRDroneToProtoTest, TestPack) {
  // SETUP
  proto::ILQRDrone ilqr_drone_msg;
  // ACTION/VERIFICATION
  pack(make_test_ilqr_drone(), &ilqr_drone_msg);
}

TEST(ILQRDroneToProtoTest, TestRoundTrip) {
  // SETUP
  const ILQRDrone& test_ilqr_drone = make_test_ilqr_drone();
  proto::ILQRDrone ilqr_drone_msg;
  // ACTION/VERIFICATION
  pack(test_ilqr_drone, &ilqr_drone_msg);
  const ILQRDrone& retrieved_ilqr_drone = unpack(ilqr_drone_msg);
  test_ilqr_drone_equality(test_ilqr_drone, retrieved_ilqr_drone);
}

TEST(ILQRDroneToProtoDeathTest, TestPackInvalid) {
  // ACTION / VERIFICATION
  EXPECT_THROW(proto::pack(make_test_ilqr_drone(), nullptr), AssertException);
}

}  // namespace resim::experiences
