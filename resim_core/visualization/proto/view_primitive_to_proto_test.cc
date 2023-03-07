
#include "resim_core/visualization/proto/view_primitive_to_proto.hh"

#include <gtest/gtest.h>

#include <random>

#include "resim_core/assert/assert.hh"
#include "resim_core/curves/d_curve.hh"
#include "resim_core/curves/proto/d_curve_fse3_to_proto.hh"
#include "resim_core/curves/proto/d_curve_se3_to_proto.hh"
#include "resim_core/testing/random_matrix.hh"
#include "resim_core/transforms/framed_group.hh"
#include "resim_core/transforms/liegroup_test_helpers.hh"
#include "resim_core/transforms/proto/se3_to_proto.hh"
#include "resim_core/transforms/se3.hh"
#include "resim_core/utils/match.hh"
#include "resim_core/utils/proto/uuid.pb.h"
#include "resim_core/utils/proto/uuid_to_proto.hh"
#include "resim_core/utils/uuid.hh"
#include "resim_core/visualization/proto/view_primitive.pb.h"
#include "resim_core/visualization/view_primitive.hh"

namespace resim::visualization {

namespace {
using transforms::FSE3;
using transforms::SE3;
using TangentVector = SE3::TangentVector;

constexpr unsigned int NUM_GROUP_POINTS = 10;

}  // namespace

template <typename T>
class ViewPrimitiveToProtoTypedTest : public ::testing::Test {
 public:
  ViewPrimitive generate_test_primitive();

 private:
  static constexpr unsigned SEED = 430;
  std::mt19937 rng{SEED};
};

template <>
ViewPrimitive ViewPrimitiveToProtoTypedTest<SE3>::generate_test_primitive() {
  const TangentVector test_tangent{testing::random_vector<TangentVector>(rng)};
  const SE3 test_se3{SE3::exp(test_tangent)};

  ViewPrimitive test_primitive{
      .id = UUID::new_uuid(),
      .payload = test_se3,
  };

  return test_primitive;
}

template <>
ViewPrimitive
ViewPrimitiveToProtoTypedTest<curves::DCurve<SE3>>::generate_test_primitive() {
  const curves::DCurve test_d_curve(
      transforms::make_test_group_elements<SE3>(NUM_GROUP_POINTS));

  ViewPrimitive test_primitive{
      .id = UUID::new_uuid(),
      .payload = test_d_curve,
  };

  return test_primitive;
}

template <>
ViewPrimitive
ViewPrimitiveToProtoTypedTest<curves::DCurve<FSE3>>::generate_test_primitive() {
  const curves::DCurve test_d_curve(
      transforms::make_test_group_elements<FSE3>(NUM_GROUP_POINTS));

  ViewPrimitive test_primitive{
      .id = UUID::new_uuid(),
      .payload = test_d_curve,
  };

  return test_primitive;
}

using PayloadTypes =
    ::testing::Types<SE3, curves::DCurve<SE3>, curves::DCurve<FSE3>>;

TYPED_TEST_SUITE(ViewPrimitiveToProtoTypedTest, PayloadTypes);

TYPED_TEST(ViewPrimitiveToProtoTypedTest, TestPack) {
  // SETUP
  const ViewPrimitive test_primitive =
      ViewPrimitiveToProtoTypedTest<TypeParam>::generate_test_primitive();
  proto::ViewPrimitive primitive_msg;

  // ACTION
  pack(test_primitive, &primitive_msg);

  // VERIFICATION
  EXPECT_EQ(test_primitive.id, unpack(primitive_msg.id()));
  match(
      test_primitive.payload,
      [&](const SE3 &test_se3) {
        EXPECT_TRUE(unpack(primitive_msg.se3()).is_approx(test_se3));
      },
      [&](const curves::DCurve<SE3> &test_d_curve_se3) {
        const auto &control_points = test_d_curve_se3.control_pts();
        const auto unpacked_control_points =
            unpack(primitive_msg.d_curve_se3()).control_pts();

        ASSERT_EQ(unpacked_control_points.size(), control_points.size());

        for (int i = 0; i < control_points.size(); i++) {
          const auto &ref_from_control = *(control_points[i].ref_from_control);
          const auto &unpacked_ref_from_control =
              *(unpacked_control_points[i].ref_from_control);

          EXPECT_TRUE(ref_from_control.is_approx(unpacked_ref_from_control));
        }
      },
      [&](const curves::DCurve<FSE3> &test_d_curve_fse3) {
        const auto &control_points = test_d_curve_fse3.control_pts();
        const auto unpacked_control_points =
            unpack(primitive_msg.d_curve_fse3()).control_pts();

        ASSERT_EQ(unpacked_control_points.size(), control_points.size());

        for (int i = 0; i < control_points.size(); i++) {
          const auto &ref_from_control = *(control_points[i].ref_from_control);
          const auto &unpacked_ref_from_control =
              *(unpacked_control_points[i].ref_from_control);

          EXPECT_TRUE(ref_from_control.is_approx(unpacked_ref_from_control));
        }
      });
}

TYPED_TEST(ViewPrimitiveToProtoTypedTest, TestRoundTrip) {
  // SETUP
  const ViewPrimitive test_primitive =
      ViewPrimitiveToProtoTypedTest<TypeParam>::generate_test_primitive();
  proto::ViewPrimitive primitive_msg;

  // ACTION
  pack(test_primitive, &primitive_msg);
  const ViewPrimitive unpacked{unpack(primitive_msg)};

  // VERIFICATION
  EXPECT_EQ(test_primitive.id, unpacked.id);
  match(
      test_primitive.payload,
      [&](const SE3 &test_se3) {
        ASSERT_TRUE(std::holds_alternative<SE3>(unpacked.payload));
        test_se3.is_approx(std::get<SE3>(unpacked.payload));
      },
      [&](const curves::DCurve<SE3> &test_d_curve_se3) {
        ASSERT_TRUE(
            std::holds_alternative<curves::DCurve<SE3>>(unpacked.payload));

        const auto &unpacked_dcurve =
            std::get<curves::DCurve<SE3>>(unpacked.payload);
        const auto &unpacked_control_points = unpacked_dcurve.control_pts();

        ASSERT_EQ(
            unpacked_control_points.size(),
            test_d_curve_se3.control_pts().size());

        for (int i = 0; i < unpacked_control_points.size(); i++) {
          const auto test_ref_from_control =
              test_d_curve_se3.control_pts().at(i).ref_from_control;
          EXPECT_TRUE(test_ref_from_control->is_approx(
              *(unpacked_control_points.at(i).ref_from_control)));
        }
      },
      [&](const curves::DCurve<FSE3> &test_d_curve_fse3) {
        ASSERT_TRUE(
            std::holds_alternative<curves::DCurve<FSE3>>(unpacked.payload));

        const auto &unpacked_dcurve =
            std::get<curves::DCurve<FSE3>>(unpacked.payload);
        const auto &unpacked_control_points = unpacked_dcurve.control_pts();

        ASSERT_EQ(
            unpacked_control_points.size(),
            test_d_curve_fse3.control_pts().size());

        for (int i = 0; i < unpacked_control_points.size(); i++) {
          const auto test_ref_from_control =
              test_d_curve_fse3.control_pts().at(i).ref_from_control;
          EXPECT_TRUE(test_ref_from_control->is_approx(
              *(unpacked_control_points.at(i).ref_from_control)));
        }
      });
}

TYPED_TEST(ViewPrimitiveToProtoTypedTest, TestPackInvalid) {
  ViewPrimitive test_primitive =
      ViewPrimitiveToProtoTypedTest<TypeParam>::generate_test_primitive();

  // ACTION/VERIFICATION
  EXPECT_THROW(proto::pack(test_primitive, nullptr), AssertException);
}

TYPED_TEST(ViewPrimitiveToProtoTypedTest, TestUnpackUnset) {
  proto::ViewPrimitive primitive_msg;
  pack(UUID::new_uuid(), primitive_msg.mutable_id());

  // ACTION / VERIFICATION
  EXPECT_FALSE(proto::detail::unpack(primitive_msg).ok());

  EXPECT_THROW(unpack(primitive_msg), AssertException);
}

}  // namespace resim::visualization
