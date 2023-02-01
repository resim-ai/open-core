
#include "resim_core/visualization/proto/view_primitive_to_proto.hh"

#include <gtest/gtest.h>

#include <random>

#include "resim_core/testing/random_matrix.hh"
#include "resim_core/transforms/proto/se3_to_proto.hh"
#include "resim_core/transforms/se3.hh"
#include "resim_core/utils/match.hh"
#include "resim_core/utils/proto/uuid.pb.h"
#include "resim_core/utils/proto/uuid_to_proto.hh"
#include "resim_core/utils/uuid.hh"
#include "resim_core/visualization/proto/view_primitive.pb.h"
#include "resim_core/visualization/view_primitive.hh"

namespace resim::visualization {

using transforms::SE3;
using TangentVector = SE3::TangentVector;

namespace {

struct ViewPrimitiveToProtoTest : public ::testing::Test {
  static constexpr unsigned SEED = 430;
  std::mt19937 rng{SEED};
  const TangentVector test_tangent{testing::random_vector<TangentVector>(rng)};
  const SE3 test_se3{SE3::exp(test_tangent)};

  const ViewPrimitive test_primitive{
      .id = UUID::new_uuid(),
      .payload = test_se3,
  };
};

}  // namespace

TEST_F(ViewPrimitiveToProtoTest, TestPack) {
  // SETUP
  proto::ViewPrimitive primitive_msg;

  // ACTION
  pack(test_primitive, &primitive_msg);

  // VERIFICATION
  EXPECT_EQ(test_primitive.id, unpack(primitive_msg.id()));
  ASSERT_EQ(primitive_msg.payload_case(), proto::ViewPrimitive::kSe3);
  EXPECT_TRUE(unpack(primitive_msg.se3()).is_approx(test_se3));
}

TEST_F(ViewPrimitiveToProtoTest, TestRoundTrip) {
  // SETUP
  proto::ViewPrimitive primitive_msg;

  // ACTION
  pack(test_primitive, &primitive_msg);
  const ViewPrimitive unpacked{unpack(primitive_msg)};

  // VERIFICATION
  EXPECT_EQ(test_primitive.id, unpacked.id);
  match(test_primitive.payload, [&](const SE3 &se3) {
    ASSERT_TRUE(std::holds_alternative<SE3>(unpacked.payload));
    se3.is_approx(std::get<SE3>(unpacked.payload));
  });
}

using ViewPrimitiveToProtoDeathTest = ViewPrimitiveToProtoTest;

TEST_F(ViewPrimitiveToProtoDeathTest, TestPackInvalid) {
  // ACTION / VERIFICATION
  EXPECT_DEATH(
      proto::pack(test_primitive, nullptr),
      "Can't pack into invalid proto!");
}

TEST_F(ViewPrimitiveToProtoDeathTest, TestUnpackUnset) {
  // SETUP
  proto::ViewPrimitive primitive_msg;
  pack(UUID::new_uuid(), primitive_msg.mutable_id());

  // ACTION / VERIFICATION
  EXPECT_FALSE(proto::detail::unpack(primitive_msg).ok());
  EXPECT_DEATH(unpack(primitive_msg), "Can't unpack unset ViewPrimitive!");
  ;
}

}  // namespace resim::visualization
