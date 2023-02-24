#include "resim_core/curves/proto/t_curve_to_proto.hh"

#include <gtest/gtest.h>

#include <ostream>

#include "resim_core/curves/proto/t_curve.pb.h"
#include "resim_core/curves/proto/t_curve_fse3_to_proto.hh"
#include "resim_core/curves/proto/t_curve_fso3_to_proto.hh"
#include "resim_core/curves/proto/t_curve_se3_to_proto.hh"
#include "resim_core/curves/proto/t_curve_so3_to_proto.hh"
#include "resim_core/curves/proto/two_jetl_fse3_to_proto.hh"
#include "resim_core/curves/proto/two_jetl_fso3_to_proto.hh"
#include "resim_core/curves/proto/two_jetl_se3_to_proto.hh"
#include "resim_core/curves/proto/two_jetl_so3_to_proto.hh"
#include "resim_core/curves/t_curve.hh"
#include "resim_core/curves/two_jet_test_helpers.hh"
#include "resim_core/transforms/framed_group.hh"
#include "resim_core/transforms/framed_group_concept.hh"
#include "resim_core/transforms/se3.hh"
#include "resim_core/transforms/so3.hh"

namespace resim::curves::proto {
using TCurveSE3 = curves::TCurve<transforms::SE3>;
using TCurveSO3 = curves::TCurve<transforms::SO3>;
using TCurveFSE3 = curves::TCurve<transforms::FSE3>;
using TCurveFSO3 = curves::TCurve<transforms::FSO3>;

// An explicit seed for deterministic generation of test objects.
constexpr unsigned int SEED = 89;

template <typename Pair>
class TCurveToProtoTests : public ::testing::Test {
 public:
  using Group = typename Pair::first_type;
  using Frame = transforms::Frame<Group::DIMS>;
  inline static const Frame REF_FRAME = Frame::new_frame();
  inline static const Frame POINT_FRAME = Frame::new_frame();
  inline static constexpr std::array<double, 3> DEFAULT_TIMES{0.0, 0.13, 0.61};

 protected:
  void SetUp() override { tj_helper_ = TwoJetTestHelper<TwoJetL<Group>>(SEED); }

  TwoJetTestHelper<TwoJetL<Group>> &tj_helper() { return tj_helper_; }

  TwoJetL<Group> test_two_jet(const Frame &into) requires
      transforms::FramedGroupType<Group> {
    TwoJetL<Group> test_tj = tj_helper().make_test_two_jet();
    Group group = test_tj.frame_from_ref();
    group.set_into(into);
    group.set_from(REF_FRAME);
    test_tj.set_frame_from_ref(group);
    return test_tj;
  }

  TCurve<Group> test_curve(const std::array<double, 3> &times) {
    TCurve<Group> test_curve;
    for (const double &t : times) {
      test_curve.append({t, tj_helper().make_test_two_jet()});
    }
    return test_curve;
  }

  TCurve<Group> test_curve(const std::array<double, 3> &times) requires
      transforms::FramedGroupType<Group> {
    TCurve<Group> test_curve;
    for (const double &t : times) {
      test_curve.append({t, test_two_jet(POINT_FRAME)});
    }
    return test_curve;
  }

  TCurve<Group> test_curve_default() { return test_curve(DEFAULT_TIMES); }

 private:
  TwoJetTestHelper<TwoJetL<Group>> tj_helper_;
};

using TCurveTypePairs = ::testing::Types<
    std::pair<transforms::SE3, TCurve_SE3>,
    std::pair<transforms::SO3, TCurve_SO3>,
    std::pair<transforms::FSO3, TCurve_FSO3>,
    std::pair<transforms::FSE3, TCurve_FSE3>>;

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
    if constexpr (transforms::FramedGroupType<typename TypeParam::first_type>) {
      EXPECT_EQ(
          original_two_jet.frame_from_ref().from(),
          retrieved_two_jet.frame_from_ref().from());
      EXPECT_EQ(
          original_two_jet.frame_from_ref().into(),
          retrieved_two_jet.frame_from_ref().into());
    }
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

  EXPECT_DEATH(
      { proto::pack(test_t_curve, nullptr); },
      "Can't pack into invalid proto!");
}

}  // namespace resim::curves::proto
