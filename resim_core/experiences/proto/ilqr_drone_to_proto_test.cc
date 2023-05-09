#include "resim_core/experiences/proto/ilqr_drone_to_proto.hh"

#include <gtest/gtest.h>

#include "resim_core/assert/assert.hh"
#include "resim_core/experiences/ilqr_drone.hh"
#include "resim_core/experiences/proto/experiences_test_helpers.hh"
#include "resim_core/experiences/proto/ilqr_drone.pb.h"

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
