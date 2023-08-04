// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.


#include "resim/visualization/proto/view_update_to_proto.hh"

#include <gtest/gtest.h>

#include "resim/assert/assert.hh"
#include "resim/transforms/se3.hh"
#include "resim/utils/match.hh"
#include "resim/utils/uuid.hh"
#include "resim/visualization/proto/view_primitive_to_proto.hh"
#include "resim/visualization/proto/view_update.pb.h"
#include "resim/visualization/testing/test_helpers.hh"
#include "resim/visualization/view_primitive.hh"
#include "resim/visualization/view_update.hh"

namespace resim::visualization {

using transforms::SE3;
using TangentVector = SE3::TangentVector;

namespace {
const TangentVector test_tangent{
    (TangentVector() << 1., 2., 3., 4., 5., 6.).finished()};

const SE3 test_se3{SE3::exp(test_tangent)};

const ViewPrimitive test_primitive{
    .id = UUID::new_uuid(),
    .payload = test_se3,
};

}  // namespace

TEST(ViewUpdateToProtoTest, TestPack) {
  // SETUP
  const ViewUpdate update{
      .primitives = {test_primitive, test_primitive},
  };

  proto::ViewUpdate update_msg;

  // ACTION
  pack(update, &update_msg);

  // VERIFICATION
  ASSERT_EQ(update_msg.primitive().size(), update.primitives.size());
  for (std::size_t ii = 0U; ii < update_msg.primitive().size(); ++ii) {
    const ViewPrimitive unpacked_primitive{
        unpack(update_msg.primitive(static_cast<int>(ii)))};
    EXPECT_TRUE(testing::primitives_equal(unpacked_primitive, test_primitive));
  }
}

TEST(ViewUpdateToProtoTest, TestRoundTrip) {
  // SETUP
  const ViewUpdate update{
      .primitives = {test_primitive, test_primitive},
  };

  proto::ViewUpdate update_msg;

  // ACTION
  pack(update, &update_msg);
  const ViewUpdate unpacked{unpack(update_msg)};

  // VERIFICATION
  ASSERT_EQ(update.primitives.size(), unpacked.primitives.size());
  for (const auto& primitive : update.primitives) {
    EXPECT_TRUE(testing::primitives_equal(primitive, test_primitive));
  }
}

TEST(ViewUpdateToProtoDeathTest, TestPackInvalid) {
  // SETUP
  const ViewUpdate update{
      .primitives = {test_primitive, test_primitive},
  };

  // ACTION / VERIFICATION
  EXPECT_THROW(proto::pack(update, nullptr), AssertException);
}

}  // namespace resim::visualization
