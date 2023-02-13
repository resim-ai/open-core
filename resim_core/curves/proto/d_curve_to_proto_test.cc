#include "resim_core/curves/proto/d_curve_to_proto.hh"

#include <gtest/gtest.h>

#include "resim_core/curves/proto/d_curve.pb.h"
#include "resim_core/curves/proto/d_curve_fse3_to_proto.hh"
#include "resim_core/curves/proto/d_curve_se3_to_proto.hh"
#include "resim_core/transforms/liegroup_test_helpers.hh"
#include "resim_core/transforms/proto/fse3_to_proto.hh"
#include "resim_core/transforms/proto/se3_to_proto.hh"
#include "resim_core/transforms/se3.hh"

namespace {
std::vector<resim::transforms::FSE3> generate_fse3_points(
    const std::vector<resim::transforms::SE3>& se3_points) {
  std::vector<resim::transforms::FSE3> points;
  points.reserve(se3_points.size());

  const auto ref_frame =
      resim::transforms::Frame<resim::transforms::FSE3::DIMS>::new_frame();
  const auto pt_frame =
      resim::transforms::Frame<resim::transforms::FSE3::DIMS>::new_frame();

  for (const auto& point : se3_points) {
    points.emplace_back(resim::transforms::FSE3(point, ref_frame, pt_frame));
  }

  return points;
}

constexpr unsigned int NUM_SE3_POINTS = 10;
}  // namespace

namespace resim::curves {

template <typename T>
class DCurveToProtoTests : public ::testing::Test {
 public:
  static std::vector<T> generate_control_points();
};

template <>
std::vector<transforms::SE3>
DCurveToProtoTests<transforms::SE3>::generate_control_points() {
  return transforms::make_test_group_elements<transforms::SE3>(NUM_SE3_POINTS);
}

// TODO(sharon): Iain will add helper function to generate fse3.
template <>
std::vector<transforms::FSE3>
DCurveToProtoTests<transforms::FSE3>::generate_control_points() {
  return generate_fse3_points(
      DCurveToProtoTests<transforms::SE3>::generate_control_points());
}

using GroupTypePairs = ::testing::Types<
    std::pair<transforms::SE3, proto::DCurve_SE3>,
    std::pair<transforms::FSE3, proto::DCurve_FSE3>>;

TYPED_TEST_SUITE(DCurveToProtoTests, GroupTypePairs);

TYPED_TEST(DCurveToProtoTests, TestPack) {
  using Group = typename TypeParam::first_type;
  using GroupMsg = typename TypeParam::second_type;

  GroupMsg msg;
  DCurve test_curve(DCurveToProtoTests<Group>::generate_control_points());

  // ACTION
  pack(test_curve, &msg);

  // VERIFICATION
  EXPECT_EQ(test_curve.control_pts().size(), msg.ref_from_control_size());

  const auto& control_points = test_curve.control_pts();
  for (int i = 0; i < msg.ref_from_control_size(); i++) {
    const auto msg_ref_from_control =
        transforms::proto::unpack(msg.ref_from_control(i));
    const auto& ref_from_control = *(control_points[i].ref_from_control);
    EXPECT_TRUE(ref_from_control.is_approx(msg_ref_from_control));
  }
}

TYPED_TEST(DCurveToProtoTests, TestRoundTrip) {
  using Group = typename TypeParam::first_type;
  using GroupMsg = typename TypeParam::second_type;

  GroupMsg msg;
  DCurve test_curve(DCurveToProtoTests<Group>::generate_control_points());

  // ACTION
  pack(test_curve, &msg);
  const DCurve unpacked_test_curve = unpack(msg);

  // VERIFICATION
  EXPECT_EQ(
      test_curve.control_pts().size(),
      unpacked_test_curve.control_pts().size());

  const auto& control_points = test_curve.control_pts();
  const auto& unpacked_control_points = unpacked_test_curve.control_pts();
  for (int i = 0; i < control_points.size(); i++) {
    const auto& ref_from_control = *(control_points[i].ref_from_control);
    const auto& unpacked_ref_from_control =
        *(unpacked_control_points[i].ref_from_control);

    EXPECT_TRUE(ref_from_control.is_approx(unpacked_ref_from_control));
  }
}

TYPED_TEST(DCurveToProtoTests, TestFail) {
  using Group = typename TypeParam::first_type;
  using GroupMsg = typename TypeParam::second_type;

  GroupMsg msg;
  DCurve test_curve(DCurveToProtoTests<Group>::generate_control_points());

  // ACTION/VERIFICATION
  EXPECT_DEATH(
      { proto::pack(test_curve, nullptr); },
      "Can't pack into invalid proto!");
}

}  // namespace resim::curves
