
#include "resim_core/visualization/proto/view_primitive_to_proto.hh"

#include <gtest/gtest.h>

#include <optional>
#include <random>
#include <variant>

#include "resim_core/actor/state/proto/trajectory_to_proto.hh"
#include "resim_core/actor/state/trajectory.hh"
#include "resim_core/assert/assert.hh"
#include "resim_core/curves/d_curve.hh"
#include "resim_core/curves/proto/d_curve_fse3_to_proto.hh"
#include "resim_core/curves/proto/d_curve_se3_to_proto.hh"
#include "resim_core/curves/proto/t_curve_fse3_to_proto.hh"
#include "resim_core/curves/t_curve.hh"
#include "resim_core/curves/two_jet.hh"
#include "resim_core/testing/random_matrix.hh"
#include "resim_core/time/timestamp.hh"
#include "resim_core/transforms/frame.hh"
#include "resim_core/transforms/framed_group.hh"
#include "resim_core/transforms/liegroup_concepts.hh"
#include "resim_core/transforms/liegroup_test_helpers.hh"
#include "resim_core/transforms/proto/frame_3_to_proto.hh"
#include "resim_core/transforms/proto/fse3_to_proto.hh"
#include "resim_core/transforms/proto/fso3_to_proto.hh"
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
using transforms::FSO3;
using transforms::SE3;
using transforms::SO3;
using Frame = transforms::Frame<3>;
constexpr time::Timestamp ZERO_TIME;
constexpr unsigned int NUM_GROUP_POINTS = 10;
const std::vector<std::optional<std::string>> NAME_RANGE = {
    "first_name",
    "second_name",
    std::nullopt};
constexpr auto TEST_FILE_NAME = "test_file_name";
constexpr int TEST_LINE_NUMBER = 42;
}  // namespace

template <typename T>
class ViewPrimitiveToProtoTypedTest : public ::testing::Test {
 public:
  T generate_test_object();
  ViewPrimitive generate_test_primitive(const std::optional<std::string> &name);

  curves::TCurve<FSE3> generate_test_t_curve() {
    auto control_point_poses =
        transforms::make_test_group_elements<FSE3>(NUM_GROUP_POINTS);
    const Frame into{Frame::new_frame()};
    const Frame from{Frame::new_frame()};
    std::vector<curves::TCurve<FSE3>::Control> control_points;
    control_points.reserve(NUM_GROUP_POINTS);
    double time = 0;
    for (FSE3 &pose : control_point_poses) {
      using TwoJet = curves::TwoJetL<FSE3>;
      pose.set_into(into);
      pose.set_from(from);
      const TwoJet point{
          pose,
          testing::random_vector<FSE3::TangentVector>(rng()),
          testing::random_vector<FSE3::TangentVector>(rng())};
      control_points.push_back(curves::TCurve<FSE3>::Control{
          .time = time,
          .point = point,
      });
      time += 1.;
    }
    return curves::TCurve<FSE3>{control_points};
  }

 protected:
  std::mt19937 &rng() { return rng_; }

 private:
  static constexpr unsigned SEED = 430;
  std::mt19937 rng_{SEED};
};

template <>
Frame ViewPrimitiveToProtoTypedTest<Frame>::generate_test_object() {
  return Frame::new_frame();
}

template <>
SE3 ViewPrimitiveToProtoTypedTest<SE3>::generate_test_object() {
  const SE3::TangentVector test_tangent{
      testing::random_vector<SE3::TangentVector>(rng())};
  return SE3::exp(test_tangent);
}

template <>
SO3 ViewPrimitiveToProtoTypedTest<SO3>::generate_test_object() {
  const SO3::TangentVector test_tangent{
      testing::random_vector<SO3::TangentVector>(rng())};
  return SO3::exp(test_tangent);
}

template <>
FSE3 ViewPrimitiveToProtoTypedTest<FSE3>::generate_test_object() {
  const Frame into{Frame::new_frame()};
  const Frame from{Frame::new_frame()};
  const FSE3::TangentVector test_tangent{
      testing::random_vector<FSE3::TangentVector>(rng())};
  return FSE3::exp(test_tangent, into, from);
}

template <>
FSO3 ViewPrimitiveToProtoTypedTest<FSO3>::generate_test_object() {
  const Frame into{Frame::new_frame()};
  const Frame from{Frame::new_frame()};
  const FSO3::TangentVector test_tangent{
      testing::random_vector<FSO3::TangentVector>(rng())};
  return FSO3::exp(test_tangent, into, from);
}

template <>
curves::DCurve<SE3>
ViewPrimitiveToProtoTypedTest<curves::DCurve<SE3>>::generate_test_object() {
  return curves::DCurve(
      transforms::make_test_group_elements<SE3>(NUM_GROUP_POINTS));
}

template <>
curves::DCurve<FSE3>
ViewPrimitiveToProtoTypedTest<curves::DCurve<FSE3>>::generate_test_object() {
  return curves::DCurve(
      transforms::make_test_group_elements<FSE3>(NUM_GROUP_POINTS));
}

template <>
curves::TCurve<FSE3>
ViewPrimitiveToProtoTypedTest<curves::TCurve<FSE3>>::generate_test_object() {
  return generate_test_t_curve();
}

template <>
actor::state::Trajectory ViewPrimitiveToProtoTypedTest<
    actor::state::Trajectory>::generate_test_object() {
  curves::TCurve<FSE3> t_curve = generate_test_t_curve();
  return actor::state::Trajectory{t_curve, ZERO_TIME};
}

template <typename T>
ViewPrimitive ViewPrimitiveToProtoTypedTest<T>::generate_test_primitive(
    const std::optional<std::string> &name) {
  ViewPrimitive test_primitive{
      .id = UUID::new_uuid(),
      .payload = generate_test_object(),
      .user_defined_name = name,
      .file_name = TEST_FILE_NAME,
      .line_number = TEST_LINE_NUMBER,
  };

  return test_primitive;
}

using PayloadTypes = ::testing::Types<
    SE3,
    SO3,
    FSE3,
    FSO3,
    curves::DCurve<SE3>,
    curves::DCurve<FSE3>,
    curves::TCurve<FSE3>,
    actor::state::Trajectory>;

TYPED_TEST_SUITE(ViewPrimitiveToProtoTypedTest, PayloadTypes);

// NOLINTBEGIN(readability-function-cognitive-complexity)
TYPED_TEST(ViewPrimitiveToProtoTypedTest, TestPack) {
  for (const std::optional<std::string> &name : NAME_RANGE) {
    // SETUP
    const ViewPrimitive test_primitive =
        ViewPrimitiveToProtoTypedTest<TypeParam>::generate_test_primitive(name);
    proto::ViewPrimitive primitive_msg;

    // ACTION
    pack(test_primitive, &primitive_msg);

    // VERIFICATION
    EXPECT_EQ(test_primitive.id, unpack(primitive_msg.id()));

    // Expect the value to be encoded as an empty field in protobuf
    // if it has no value.
    EXPECT_EQ(name.value_or(""), primitive_msg.user_defined_name());
    EXPECT_EQ(TEST_FILE_NAME, primitive_msg.file_name());
    EXPECT_EQ(TEST_LINE_NUMBER, primitive_msg.line_number());
    match(
        test_primitive.payload,
        [&](const Frame &test_frame) {
          EXPECT_TRUE(unpack(primitive_msg.frame()) == test_frame);
        },
        [&](const SE3 &test_se3) {
          EXPECT_TRUE(unpack(primitive_msg.se3()).is_approx(test_se3));
        },
        [&](const SO3 &test_so3) {
          EXPECT_TRUE(unpack(primitive_msg.so3()).is_approx(test_so3));
        },
        [&](const FSE3 &test_fse3) {
          EXPECT_TRUE(unpack(primitive_msg.fse3()).is_approx(test_fse3));
        },
        [&](const FSO3 &test_fso3) {
          EXPECT_TRUE(unpack(primitive_msg.fso3()).is_approx(test_fso3));
        },
        [&](const curves::DCurve<SE3> &test_d_curve_se3) {
          const auto &control_points = test_d_curve_se3.control_pts();
          const auto unpacked_control_points =
              unpack(primitive_msg.d_curve_se3()).control_pts();

          ASSERT_EQ(unpacked_control_points.size(), control_points.size());

          for (int i = 0; i < control_points.size(); i++) {
            const auto &ref_from_control =
                *(control_points[i].ref_from_control);
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
            const auto &ref_from_control =
                *(control_points[i].ref_from_control);
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
        },
        [&](const actor::state::Trajectory &test_trajectory) {
          const auto &control_points = test_trajectory.curve().control_pts();
          const auto &unpacked_trajectory = unpack(primitive_msg.trajectory());
          const auto unpacked_control_points =
              unpacked_trajectory.curve().control_pts();
          EXPECT_EQ(
              unpacked_trajectory.start_time(),
              test_trajectory.start_time());
          ASSERT_EQ(control_points.size(), unpacked_control_points.size());
          for (int i = 0; i < control_points.size(); i++) {
            const auto &control = control_points.at(i);
            const auto &unpacked_control = unpacked_control_points.at(i);
            EXPECT_EQ(control.time, unpacked_control.time);
            EXPECT_TRUE(control.point.is_approx(unpacked_control.point));
          }
        });
  }
}

TYPED_TEST(ViewPrimitiveToProtoTypedTest, TestRoundTrip) {
  for (const std::optional<std::string> &name : NAME_RANGE) {
    // SETUP
    const ViewPrimitive test_primitive =
        ViewPrimitiveToProtoTypedTest<TypeParam>::generate_test_primitive(name);
    proto::ViewPrimitive primitive_msg;

    // ACTION
    pack(test_primitive, &primitive_msg);
    const ViewPrimitive unpacked{unpack(primitive_msg)};

    // VERIFICATION
    EXPECT_EQ(test_primitive.id, unpacked.id);
    EXPECT_EQ(test_primitive.user_defined_name, unpacked.user_defined_name);
    EXPECT_EQ(test_primitive.file_name, unpacked.file_name);
    EXPECT_EQ(test_primitive.line_number, unpacked.line_number);
    match(
        test_primitive.payload,
        [&](const Frame &test_frame) {
          ASSERT_TRUE(std::holds_alternative<Frame>(unpacked.payload));
          ASSERT_EQ(test_frame, std::get<Frame>(unpacked.payload));
        },
        [&](const SE3 &test_se3) {
          ASSERT_TRUE(std::holds_alternative<SE3>(unpacked.payload));
          ASSERT_TRUE(test_se3.is_approx(std::get<SE3>(unpacked.payload)));
        },
        [&](const SO3 &test_so3) {
          ASSERT_TRUE(std::holds_alternative<SO3>(unpacked.payload));
          ASSERT_TRUE(test_so3.is_approx(std::get<SO3>(unpacked.payload)));
        },
        [&](const FSE3 &test_fse3) {
          ASSERT_TRUE(std::holds_alternative<FSE3>(unpacked.payload));
          ASSERT_TRUE(test_fse3.is_approx(std::get<FSE3>(unpacked.payload)));
        },
        [&](const FSO3 &test_fso3) {
          ASSERT_TRUE(std::holds_alternative<FSO3>(unpacked.payload));
          ASSERT_TRUE(test_fso3.is_approx(std::get<FSO3>(unpacked.payload)));
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
        },
        [&](const actor::state::Trajectory &test_trajectory) {
          ASSERT_TRUE(std::holds_alternative<actor::state::Trajectory>(
              unpacked.payload));
          const actor::state::Trajectory &unpacked_trajectory{
              std::get<actor::state::Trajectory>(unpacked.payload)};
          EXPECT_EQ(
              unpacked_trajectory.start_time(),
              test_trajectory.start_time());
          const auto &control_points = test_trajectory.curve().control_pts();
          const auto &unpacked_control_points =
              unpacked_trajectory.curve().control_pts();
          ASSERT_EQ(control_points.size(), unpacked_control_points.size());

          for (int i = 0; i < control_points.size(); i++) {
            const auto &control = control_points.at(i);
            const auto &unpacked_control = unpacked_control_points.at(i);
            EXPECT_EQ(control.time, unpacked_control.time);
            EXPECT_TRUE(control.point.is_approx(unpacked_control.point));
          }
        });
  }
}
// NOLINTEND(readability-function-cognitive-complexity)

TYPED_TEST(ViewPrimitiveToProtoTypedTest, TestPackInvalid) {
  ViewPrimitive test_primitive =
      ViewPrimitiveToProtoTypedTest<TypeParam>::generate_test_primitive(
          std::nullopt);  // We use the default name

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
