#include "resim_core/curves/proto/t_curve_to_proto.hh"

#include <gtest/gtest.h>

#include <ostream>

#include "resim_core/assert/assert.hh"
#include "resim_core/curves/proto/t_curve.pb.h"
#include "resim_core/curves/proto/t_curve_se3_to_proto.hh"
#include "resim_core/curves/proto/t_curve_so3_to_proto.hh"
#include "resim_core/curves/proto/two_jetl_se3_to_proto.hh"
#include "resim_core/curves/proto/two_jetl_so3_to_proto.hh"
#include "resim_core/curves/t_curve.hh"
#include "resim_core/curves/t_curve_test_helpers.hh"
#include "resim_core/transforms/se3.hh"
#include "resim_core/transforms/so3.hh"

namespace resim::curves::proto {
using TCurveSE3 = curves::TCurve<transforms::SE3>;
using TCurveSO3 = curves::TCurve<transforms::SO3>;

// An explicit seed for deterministic generation of test objects.
constexpr unsigned int SEED = 89;

template <typename Pair>
class TCurveToProtoTests : public ::testing::Test {
 public:
  using Group = typename Pair::first_type;
  using Frame = transforms::Frame<Group::DIMS>;
  inline static const Frame REF_FRAME = Frame::new_frame();
  inline static const Frame POINT_FRAME = Frame::new_frame();
  inline static const std::vector<double> DEFAULT_TIMES{0.0, 0.13, 0.61};

 protected:
  void SetUp() override { t_curve_helper_ = TCurveTestHelper<Group>(SEED); }
  TCurveTestHelper<Group> &t_curve_helper() { return t_curve_helper_; }

  TCurve<Group> test_curve_default() {
    return t_curve_helper().make_t_curve(DEFAULT_TIMES);
  }

 private:
  TCurveTestHelper<Group> t_curve_helper_;
};

using TCurveTypePairs = ::testing::Types<
    std::pair<transforms::SE3, TCurve_SE3>,
    std::pair<transforms::SO3, TCurve_SO3>>;

TYPED_TEST_SUITE(TCurveToProtoTests, TCurveTypePairs);

// NOLINTBEGIN(readability-function-cognitive-complexity)
TYPED_TEST(TCurveToProtoTests, TestPack) {
  // SETUP
  using Group = typename TypeParam::first_type;
  using TCurveMsg = typename TypeParam::second_type;
  TCurveMsg msg;
  TCurve<Group> test_element = this->test_curve_default();
  const std::vector<typename TCurve<Group>::Control> &test_points =
      test_element.control_pts();
  // ACTION
  pack(test_element, &msg);
  const TCurve<Group> retrieved_t_curve = unpack(msg);
  const std::vector<typename TCurve<Group>::Control> &retrieved_points =
      retrieved_t_curve.control_pts();
  // VERIFICATION
  for (unsigned int i = 0; i < test_points.size(); ++i) {
    EXPECT_DOUBLE_EQ(
        retrieved_points[i].time,
        TestFixture::DEFAULT_TIMES.at(i));
    const TwoJetL<Group> &retrieved_two_jet = retrieved_points[i].point;
    const TwoJetL<Group> &original_two_jet = test_points[i].point;
    EXPECT_TRUE(original_two_jet.is_approx(retrieved_two_jet));
  }
}
// NOLINTEND(readability-function-cognitive-complexity)

TYPED_TEST(TCurveToProtoTests, TestRoundTrip) {
  // SETUP
  using Group = typename TypeParam::first_type;
  using TCurveMsg = typename TypeParam::second_type;
  TCurveMsg msg;
  TCurve<Group> test_t_curve = this->test_curve_default();

  // ACTION
  pack(test_t_curve, &msg);
  const TCurve<Group> retrieved_t_curve = unpack(msg);

  // VERIFICATION
  EXPECT_EQ(
      test_t_curve.control_pts().size(),
      retrieved_t_curve.control_pts().size());
  const auto &control_points = test_t_curve.control_pts();
  const auto &unpacked_control_points = retrieved_t_curve.control_pts();
  for (int i = 0; i < control_points.size(); i++) {
    const auto time = control_points[i].time;
    const auto unpacked_time = unpacked_control_points[i].time;
    EXPECT_DOUBLE_EQ(time, unpacked_time);
    const auto point = control_points[i].point;
    const auto unpacked_point = unpacked_control_points[i].point;
    EXPECT_TRUE(point.is_approx(unpacked_point));
  }
}

template <typename Pair>
using TCurveToProtoDeathTests = TCurveToProtoTests<Pair>;
TYPED_TEST_SUITE(TCurveToProtoDeathTests, TCurveTypePairs);

TYPED_TEST(TCurveToProtoDeathTests, TestPackNull) {
  // SETUP
  using Group = typename TypeParam::first_type;
  TCurve<Group> test_t_curve = this->test_curve_default();
  // ACTION/VERIFICATION

  EXPECT_THROW({ proto::pack(test_t_curve, nullptr); }, AssertException);
}

}  // namespace resim::curves::proto
