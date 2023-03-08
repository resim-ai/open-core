
#include "resim_core/visualization/proto/view_primitive_to_proto.hh"

#include <gtest/gtest.h>

#include <random>
#include <variant>

#include "resim_core/assert/assert.hh"
#include "resim_core/curves/d_curve.hh"
#include "resim_core/curves/proto/d_curve_fse3_to_proto.hh"
#include "resim_core/curves/proto/d_curve_se3_to_proto.hh"
#include "resim_core/curves/proto/t_curve_fse3_to_proto.hh"
#include "resim_core/curves/t_curve.hh"
#include "resim_core/curves/two_jet.hh"
#include "resim_core/testing/random_matrix.hh"
#include "resim_core/transforms/frame.hh"
#include "resim_core/transforms/framed_group.hh"
#include "resim_core/transforms/liegroup_concepts.hh"
#include "resim_core/transforms/liegroup_test_helpers.hh"
#include "resim_core/transforms/proto/se3_to_proto.hh"
#include "resim_core/transforms/proto/so3_to_proto.hh"
#include "resim_core/transforms/se3.hh"
#include "resim_core/transforms/so3.hh"
#include "resim_core/utils/match.hh"
#include "resim_core/utils/proto/uuid.pb.h"
#include "resim_core/utils/proto/uuid_to_proto.hh"
#include "resim_core/utils/uuid.hh"
#include "resim_core/visualization/curve/test_helpers.hh"
#include "resim_core/visualization/proto/view_primitive.pb.h"
#include "resim_core/visualization/view_primitive.hh"

namespace resim::visualization {

namespace {
using transforms::FSE3;
using transforms::SE3;
using transforms::SO3;
using Frame3 = transforms::Frame<3>;

constexpr unsigned int NUM_GROUP_POINTS = 10;

}  // namespace

template <typename T>
class ViewPrimitiveToProtoTypedTest : public ::testing::Test {
 public:
  ViewPrimitive generate_test_primitive();

 private:
  static constexpr unsigned SEED = 430;
  std::mt19937 rng_{SEED};
};

template <>
ViewPrimitive ViewPrimitiveToProtoTypedTest<SE3>::generate_test_primitive() {
  const SE3::TangentVector test_tangent{
      testing::random_vector<SE3::TangentVector>(rng_)};
  const SE3 test_se3{SE3::exp(test_tangent)};

  ViewPrimitive test_primitive{
      .id = UUID::new_uuid(),
      .payload = test_se3,
  };

  return test_primitive;
}

template <>
ViewPrimitive ViewPrimitiveToProtoTypedTest<SO3>::generate_test_primitive() {
  const SO3::TangentVector test_tangent{
      testing::random_vector<SO3::TangentVector>(rng_)};
  const SO3 test_so3{SO3::exp(test_tangent)};

  ViewPrimitive test_primitive{
      .id = UUID::new_uuid(),
      .payload = test_so3,
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

template <>
ViewPrimitive
ViewPrimitiveToProtoTypedTest<curves::TCurve<FSE3>>::generate_test_primitive() {
  auto control_point_poses =
      transforms::make_test_group_elements<FSE3>(NUM_GROUP_POINTS);

  const Frame3 into{Frame3::new_frame()};
  const Frame3 from{Frame3::new_frame()};

  std::vector<curves::TCurve<FSE3>::Control> control_points;
  control_points.reserve(NUM_GROUP_POINTS);
  double time = 0;
  for (FSE3 &pose : control_point_poses) {
    using TwoJet = curves::TwoJetL<FSE3>;

    pose.set_into(into);
    pose.set_from(from);
    const TwoJet point{
        pose,
        testing::random_vector<FSE3::TangentVector>(rng_),
        testing::random_vector<FSE3::TangentVector>(rng_)};

    control_points.push_back(curves::TCurve<FSE3>::Control{
        .time = time,
        .point = point,
    });
    time += 1.;
  }

  ViewPrimitive test_primitive{
      .id = UUID::new_uuid(),
      .payload = curves::TCurve<FSE3>{control_points}};

  return test_primitive;
}

using PayloadTypes = ::testing::Types<
    SE3,
    SO3,
    curves::DCurve<SE3>,
    curves::DCurve<FSE3>,
    curves::TCurve<FSE3>>;

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
      [&](const transforms::SO3 &test_so3) {
        EXPECT_TRUE(unpack(primitive_msg.so3()).is_approx(test_so3));
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
      },
      [&](const curves::TCurve<FSE3> &test_t_curve_fse3) {
        const auto &control_points = test_t_curve_fse3.control_pts();
        const auto unpacked_control_points =
            unpack(primitive_msg.t_curve_fse3()).control_pts();

        ASSERT_EQ(control_points.size(), unpacked_control_points.size());

        for (int i = 0; i < control_points.size(); i++) {
          const auto &control = control_points.at(i);
          const auto &unpacked_control = unpacked_control_points.at(i);
          EXPECT_EQ(control.time, unpacked_control.time);
          EXPECT_TRUE(control.point.is_approx(unpacked_control.point));
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
      [&](const transforms::SO3 &test_so3) {
        ASSERT_TRUE(std::holds_alternative<SO3>(unpacked.payload));
        test_so3.is_approx(std::get<SO3>(unpacked.payload));
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
      },
      [&](const curves::TCurve<FSE3> &test_t_curve_fse3) {
        ASSERT_TRUE(
            std::holds_alternative<curves::TCurve<FSE3>>(unpacked.payload));
        const auto &control_points = test_t_curve_fse3.control_pts();
        const curves::TCurve<FSE3> unpacked_t_curve{
            std::get<curves::TCurve<FSE3>>(unpacked.payload)};
        const auto &unpacked_control_points = unpacked_t_curve.control_pts();

        ASSERT_EQ(control_points.size(), unpacked_control_points.size());

        for (int i = 0; i < control_points.size(); i++) {
          const auto &control = control_points.at(i);
          const auto &unpacked_control = unpacked_control_points.at(i);
          EXPECT_EQ(control.time, unpacked_control.time);
          EXPECT_TRUE(control.point.is_approx(unpacked_control.point));
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
