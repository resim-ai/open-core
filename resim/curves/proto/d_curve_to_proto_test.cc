// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/curves/proto/d_curve_to_proto.hh"

#include <gtest/gtest.h>

#include "resim/assert/assert.hh"
#include "resim/curves/proto/d_curve.pb.h"
#include "resim/curves/proto/d_curve_se3_to_proto.hh"
#include "resim/transforms/liegroup_test_helpers.hh"
#include "resim/transforms/proto/se3_to_proto.hh"
#include "resim/transforms/se3.hh"

namespace {
constexpr unsigned int NUM_GROUP_POINTS = 10;
}  // namespace

namespace resim::curves {

template <typename T>
class DCurveToProtoTests : public ::testing::Test {
 public:
  static std::vector<T> generate_control_points();
};

template <typename T>
std::vector<T> DCurveToProtoTests<T>::generate_control_points() {
  return transforms::make_test_group_elements<T>(NUM_GROUP_POINTS);
}

using GroupTypePairs =
    ::testing::Types<std::pair<transforms::SE3, proto::DCurve_SE3>>;

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
  EXPECT_THROW({ proto::pack(test_curve, nullptr); }, AssertException);
}

}  // namespace resim::curves
