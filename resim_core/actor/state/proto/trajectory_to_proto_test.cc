#include "resim_core/actor/state/proto/trajectory_to_proto.hh"

#include <gtest/gtest.h>

#include <random>

#include "resim_core/actor/state/trajectory.hh"
#include "resim_core/assert/assert.hh"
#include "resim_core/curves/proto/t_curve_fse3_to_proto.hh"
#include "resim_core/curves/t_curve.hh"
#include "resim_core/curves/t_curve_test_helpers.hh"
#include "resim_core/curves/two_jet.hh"
#include "resim_core/time/random_duration.hh"
#include "resim_core/time/timestamp.hh"
#include "resim_core/utils/inout.hh"

namespace resim::actor::state {

namespace {
using FSE3 = transforms::FSE3;
using TCurve = curves::TCurve<FSE3>;
using Frame = transforms::Frame<FSE3::DIMS>;
// An explicit seed for deterministic generation of test objects.
constexpr unsigned int SEED = 347;
}  // namespace

class TrajectoryToProtoTests : public ::testing::Test {
 protected:
  inline static const std::vector<double> DEFAULT_TIMES{0.15, 0.71, 1.33};
  inline static const time::Timestamp ZERO_TIME;

  void SetUp() override {
    t_curve_helper_ = curves::TCurveTestHelper<FSE3>(SEED);
  }
  curves::TCurveTestHelper<FSE3> &t_curve_helper() { return t_curve_helper_; }
  std::mt19937 &rng() { return rng_; }

  TCurve test_curve_default() {
    return t_curve_helper().make_t_curve(DEFAULT_TIMES);
  }

  // Generate a test trajectory from three control points at the default
  // timestamps, but with random states.
  Trajectory default_test_trajectory() {
    return Trajectory(test_curve_default(), ZERO_TIME);
  }

 private:
  curves::TCurveTestHelper<FSE3> t_curve_helper_;
  std::mt19937 rng_{SEED};
};

TEST_F(TrajectoryToProtoTests, TestPack) {
  // SETUP
  // We use a random timestamp to get confidence in the time conversions
  const time::Timestamp random_timestamp{time::random_duration(InOut{rng()})};
  const TCurve test_curve = test_curve_default();
  const Trajectory standard_test_trajectory =
      Trajectory(test_curve, random_timestamp);
  const std::vector<typename TCurve::Control> &test_points =
      test_curve.control_pts();
  proto::Trajectory proto_trajectory;
  // ACTION
  pack(standard_test_trajectory, &proto_trajectory);
  const TCurve retrieved_t_curve = unpack(proto_trajectory.curve());
  const std::vector<typename TCurve::Control> &retrieved_points =
      retrieved_t_curve.control_pts();
  // VERIFICATION
  // Ensure that start times match
  EXPECT_EQ(
      proto_trajectory.start_time().seconds() * std::nano::den +
          proto_trajectory.start_time().nanos(),
      random_timestamp.time_since_epoch().count());
  // Ensure that underlying TCurves match
  for (unsigned int i = 0; i < test_points.size(); ++i) {
    EXPECT_DOUBLE_EQ(retrieved_points[i].time, DEFAULT_TIMES.at(i));
    const curves::TwoJetL<FSE3> &retrieved_two_jet = retrieved_points[i].point;
    const curves::TwoJetL<FSE3> &original_two_jet = test_points[i].point;
    EXPECT_TRUE(original_two_jet.is_approx(retrieved_two_jet));
    EXPECT_EQ(
        original_two_jet.frame_from_ref().from(),
        retrieved_two_jet.frame_from_ref().from());
    EXPECT_EQ(
        original_two_jet.frame_from_ref().into(),
        retrieved_two_jet.frame_from_ref().into());
  }
}

TEST_F(TrajectoryToProtoTests, TestRoundTrip) {
  // SETUP
  Trajectory standard_test_trajectory = this->default_test_trajectory();
  proto::Trajectory proto_trajectory;
  // ACTION
  pack(standard_test_trajectory, &proto_trajectory);
  Trajectory result_trajectory = unpack(proto_trajectory);
  // VERIFICATION
  EXPECT_EQ(
      standard_test_trajectory.start_time(),
      result_trajectory.start_time());
  const auto &test_t_curve = standard_test_trajectory.curve();
  const auto &retrieved_t_curve = result_trajectory.curve();
  EXPECT_EQ(
      test_t_curve.control_pts().size(),
      retrieved_t_curve.control_pts().size());
  const auto &control_points = test_t_curve.control_pts();
  const auto &unpacked_control_points = retrieved_t_curve.control_pts();
  for (int i = 0; i < control_points.size(); i++) {
    const auto &time = control_points[i].time;
    const auto &unpacked_time = unpacked_control_points[i].time;
    EXPECT_DOUBLE_EQ(time, unpacked_time);
    const auto &point = control_points[i].point;
    const auto &unpacked_point = unpacked_control_points[i].point;
    EXPECT_TRUE(point.is_approx(unpacked_point));
  }
}

using TrajectoryToProtoDeathTests = TrajectoryToProtoTests;

TEST_F(TrajectoryToProtoDeathTests, InvalidProto) {
  // ACTION/VERIFICATION
  EXPECT_THROW(
      { proto::pack(this->default_test_trajectory(), nullptr); },
      AssertException);
}

}  // namespace resim::actor::state
