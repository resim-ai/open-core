#include "resim/actor/state/trajectory.hh"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <random>

#include "resim/actor/state/rigid_body_state.hh"
#include "resim/assert/assert.hh"
#include "resim/curves/t_curve.hh"
#include "resim/curves/two_jet.hh"
#include "resim/testing/random_matrix.hh"
#include "resim/time/timestamp.hh"
#include "resim/transforms/frame.hh"
#include "resim/transforms/se3.hh"

namespace resim::actor::state {

namespace {
using SE3 = transforms::SE3;
using Frame = transforms::Frame<SE3::DIMS>;
}  // namespace

class TrajectoryTests : public ::testing::Test {
 protected:
  // Define some useful constants for use in tests.
  inline static const Frame REF_FRAME = Frame::new_frame();
  inline static const Frame BOD_FRAME = Frame::new_frame();
  inline static constexpr unsigned int NUM_CTRL = 3;
  inline static constexpr unsigned int NUM_SAMPLES = 10;
  inline static constexpr std::array<double, NUM_CTRL> DEFAULT_TIMES{
      0.15,
      0.71,
      1.33};
  inline static const time::Timestamp ZERO_TIME;

  // Provide time::Timestamp representations of DEFAULT_TIMES
  static constexpr std::array<time::Timestamp, NUM_CTRL> timestamps() {
    std::array<time::Timestamp, NUM_CTRL> timestamps;
    std::transform(
        DEFAULT_TIMES.cbegin(),
        DEFAULT_TIMES.cend(),
        timestamps.begin(),
        [](const double t) { return ZERO_TIME + time::as_duration(t); });
    return timestamps;
  }

  // Create some sample Timestamps that are evenly spaced in the interval of
  // the trajectory.
  static constexpr std::array<time::Timestamp, NUM_SAMPLES>
  timestamp_samples() {
    std::array<time::Timestamp, NUM_SAMPLES> timestamp_samples;
    // Be careful not to overflow the interval by stopping EPS before the end.
    // Note this also guarantees an unsampled point at .end_time() which is
    // used for testing negative cases below.
    constexpr double EPS = 1E-6;
    for (unsigned int i = 0; i < NUM_SAMPLES; ++i) {
      const double delta =
          i * (DEFAULT_TIMES.back() - DEFAULT_TIMES.front() - EPS) /
          NUM_SAMPLES;
      timestamp_samples.at(i) = ZERO_TIME +
                                time::as_duration(DEFAULT_TIMES.front()) +
                                time::as_duration(delta);
    }
    return timestamp_samples;
  }

  void SetUp() override {
    constexpr unsigned int SEED = 401;
    rng_ = std::mt19937(SEED);
  }

  // Generate a random TangentVector for building LieGroups and derivatives.
  SE3::TangentVector test_vector() {
    return testing::random_vector<SE3::TangentVector>(rng_);
  }

  // Generate a random TwoJet given reference and body frames.
  curves::TwoJetR<SE3> test_two_jet(const Frame &ref, const Frame &bod) {
    return curves::TwoJetR<SE3>(
        SE3::exp(test_vector(), ref, bod),
        test_vector(),
        test_vector());
  }

  // Generate a random TwoJet with default frames.
  curves::TwoJetR<SE3> test_two_jet() {
    return test_two_jet(REF_FRAME, BOD_FRAME);
  }

  // Generate a random RigidBodyState with default frames.
  RigidBodyState<SE3> test_state() {
    return RigidBodyState<SE3>(test_two_jet());
  }

  // Generate a test trajectory from three control points at the default
  // timestamps, but with random states.
  Trajectory test_trajectory() {
    Trajectory test_trajectory;
    for (const auto &ts : timestamps()) {
      test_trajectory.append({ts, test_state()});
    }
    return test_trajectory;
  }

 private:
  std::mt19937 rng_;
};

TEST_F(TrajectoryTests, TCurveConstructor) {
  // SETUP
  Trajectory standard_test_trajectory = this->test_trajectory();
  const curves::TCurve<SE3> &underlying_curve =
      standard_test_trajectory.curve();
  // We assume exactly three control points in this test so let's check.
  ASSERT_EQ(underlying_curve.control_pts().size(), NUM_CTRL);
  // ACTION
  // Create a copy of the trajectory from the underlying TCurve.
  Trajectory tcurve_test_trajectory(
      underlying_curve,
      standard_test_trajectory.start_time());
  // VERIFICATION
  // Sample both trajectories and confirm the samples match.
  for (const auto &ts : TrajectoryTests::timestamp_samples()) {
    EXPECT_TRUE(
        standard_test_trajectory.point_at(ts).ref_from_body_two_jet().is_approx(
            tcurve_test_trajectory.point_at(ts).ref_from_body_two_jet()));
    // Confirm sample differs from trajectory endpoint to cover the edge case
    // of all states are the same.
    EXPECT_FALSE(
        standard_test_trajectory.point_at(ts).ref_from_body_two_jet().is_approx(
            tcurve_test_trajectory.point_at(tcurve_test_trajectory.end_time())
                .ref_from_body_two_jet()));
  }
}

TEST_F(TrajectoryTests, Construction) {
  // SETUP
  Trajectory test_trajectory = this->test_trajectory();
  // ACTION / VERIFICATION
  // Trajectory has expected numnber of control points.
  EXPECT_EQ(test_trajectory.curve().control_pts().size(), NUM_CTRL);
  // Start time matches default timestamps
  EXPECT_DOUBLE_EQ(
      time::as_seconds(test_trajectory.start_time() - this->ZERO_TIME),
      this->DEFAULT_TIMES.front());
  // End time matches default timestamps
  EXPECT_DOUBLE_EQ(
      time::as_seconds(test_trajectory.end_time() - this->ZERO_TIME),
      this->DEFAULT_TIMES.back());
  // Duration  matches default timestamps
  EXPECT_DOUBLE_EQ(
      time::as_seconds(test_trajectory.time_duration()),
      (this->DEFAULT_TIMES.back() - this->DEFAULT_TIMES.front()));
  // Confirm end time differs from default start time to cover the
  // all-equal & all-zero edge case.
  EXPECT_NE(
      time::as_seconds(test_trajectory.end_time() - this->ZERO_TIME),
      this->DEFAULT_TIMES.front());
}

TEST_F(TrajectoryTests, CopyAndSample) {
  // SETUP
  Trajectory test_traj_a = this->test_trajectory();
  // We assume exactly three control points in this test so let's check.
  ASSERT_EQ(test_traj_a.curve().control_pts().size(), 3);
  // ACTION
  const auto timestamps = TrajectoryTests::timestamps();
  // Create a copy of the trajectory from individual control points.
  Trajectory test_traj_b{
      {timestamps.at(0), test_traj_a.point_at(timestamps.at(0))},
      {timestamps.at(1), test_traj_a.point_at(timestamps.at(1))},
      {timestamps.at(2), test_traj_a.point_at(timestamps.at(2))}};
  // VERIFICATION
  // Sample both trajectories and confirm the samples match.
  for (const auto &ts : TrajectoryTests::timestamp_samples()) {
    EXPECT_TRUE(test_traj_a.point_at(ts).ref_from_body_two_jet().is_approx(
        test_traj_b.point_at(ts).ref_from_body_two_jet()));
    // Confirm sample differs from trajectory endpoint to cover the edge case
    // of all states are the same.
    EXPECT_FALSE(test_traj_a.point_at(ts).ref_from_body_two_jet().is_approx(
        test_traj_b.point_at(test_traj_b.end_time()).ref_from_body_two_jet()));
  }
}

TEST_F(TrajectoryTests, SampleByTimestampAndDuration) {
  // SETUP
  Trajectory test_trajectory = this->test_trajectory();
  // ACTION / VERIFICATION
  // Sample the test trajectory by both timestamp and duration and confirm
  // that the samples match.
  for (const auto &ts : TrajectoryTests::timestamp_samples()) {
    const time::Duration d = ts - test_trajectory.start_time();
    const auto state_a = test_trajectory.point_at(ts);
    const auto state_b = test_trajectory.point_at(d);
    const auto end_state = test_trajectory.point_at(test_trajectory.end_time());
    EXPECT_TRUE(state_a.ref_from_body_two_jet().is_approx(
        state_b.ref_from_body_two_jet()));
    // Confirm sample differs from trajectory endpoint to cover the edge case
    // of all states are the same.
    EXPECT_FALSE(state_b.ref_from_body_two_jet().is_approx(
        end_state.ref_from_body_two_jet()));
  }
}

TEST_F(TrajectoryTests, CompareCurveSamples) {
  // SETUP
  Trajectory test_trajectory = this->test_trajectory();
  const double end_t = time::as_seconds(test_trajectory.time_duration());
  // ACTION / VERIFICATION
  // Sample the test trajectory and the underlying TCurve and confirm
  // that the samples match.
  for (const auto &ts : TrajectoryTests::timestamp_samples()) {
    const double t = time::as_seconds(ts - test_trajectory.start_time());
    EXPECT_TRUE(test_trajectory.point_at(ts).ref_from_body_two_jet().is_approx(
        test_trajectory.curve().point_at(t).right_two_jet()));
    // Confirm sample differs from trajectory endpoint to cover the edge case
    // of all states are the same.
    EXPECT_FALSE(test_trajectory.point_at(ts).ref_from_body_two_jet().is_approx(
        test_trajectory.curve().point_at(end_t).right_two_jet()));
  }
}

TEST_F(TrajectoryTests, Frames) {
  // SETUP
  Trajectory test_trajectory = this->test_trajectory();
  // ACTION / VERIFICATION
  // Confirm the frames are the default.
  EXPECT_EQ(test_trajectory.reference_frame(), this->REF_FRAME);
  EXPECT_EQ(test_trajectory.body_frame(), this->BOD_FRAME);
  // Cover the edge case where all frames are the same.
  EXPECT_NE(test_trajectory.body_frame(), this->REF_FRAME);
}

using TrajectoryDeathTests = TrajectoryTests;

TEST_F(TrajectoryDeathTests, InvalidFrames) {
  // SETUP
  Trajectory test_trajectory = this->test_trajectory();
  const Frame IMPOSTER_FRAME = Frame::new_frame();
  constexpr double additional_duration_s = 0.21;
  // ACTION
  const auto next_ts =
      test_trajectory.end_time() + time::as_duration(additional_duration_s);
  auto next_point_a = test_two_jet(TrajectoryTests::REF_FRAME, IMPOSTER_FRAME);
  auto next_point_b = test_two_jet(IMPOSTER_FRAME, TrajectoryTests::BOD_FRAME);
  // VERIFICATION
  // Confirm that introducing an imposter frame generates the expected errors.
  EXPECT_THROW(
      {
        test_trajectory.append({next_ts, RigidBodyState<SE3>(next_point_a)});
      },
      AssertException);
  EXPECT_THROW(
      {
        test_trajectory.append({next_ts, RigidBodyState<SE3>(next_point_b)});
      },
      AssertException);
}

}  // namespace resim::actor::state
