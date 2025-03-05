// Copyright 2025 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/visualization/curve/poses_in_frame_from_t_curve.hh"

#include <foxglove/Pose.pb.h>
#include <foxglove/PosesInFrame.pb.h>
#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>

#include "resim/curves/t_curve.hh"
#include "resim/curves/test_helpers.hh"
#include "resim/time/timestamp.hh"
#include "resim/transforms/frame.hh"
#include "resim/transforms/se3.hh"
#include "resim/utils/uuid.hh"
#include "resim/visualization/foxglove/pose_to_foxglove.hh"

namespace resim::visualization::curve {

using transforms::SE3;
using TCurve = curves::TCurve<SE3>;
using Frame = transforms::Frame<SE3::DIMS>;
using std::literals::chrono_literals::operator""ms;

namespace {

// A helper which confirms that a given SE3 exactly matches the given pose.
void expect_pose_matches(
    const ::foxglove::Pose &pose,
    const SE3 &ref_from_frame) {
  ::foxglove::Pose repacked_pose;
  foxglove::pack_into_foxglove(ref_from_frame, &repacked_pose);
  EXPECT_EQ(pose.SerializeAsString(), repacked_pose.SerializeAsString());
}

// A helper which verifies that the poses are as expected in the given
// PosesInFrame given the input information used to generate it.
void expect_poses_correct(
    const TCurve &curve,
    const time::Timestamp start_time,
    const time::Timestamp end_time,
    const time::Timestamp time,
    const time::Duration pose_period,
    const ::foxglove::PosesInFrame &poses) {
  const time::Timestamp first_sample_time{
      start_time +
      time::Duration{(time - start_time).count() % pose_period.count()}};

  const int64_t expected_steps =
      (end_time - first_sample_time).count() / pose_period.count() + 1;

  EXPECT_EQ(expected_steps, poses.poses().size());

  for (int ii = 0; ii < expected_steps; ++ii) {
    const time::Timestamp sample_time{first_sample_time + ii * pose_period};
    const SE3 expected_ref_from_frame{
        curve.point_at(time::as_seconds(sample_time.time_since_epoch()))
            .frame_from_ref()
            .inverse()};
    expect_pose_matches(poses.poses(ii), expected_ref_from_frame);
  }
}

}  // namespace

// Test that we correctly sample a test curve.
TEST(PosesInFrameFromCurveTest, TestPosesInFrameFromCurve) {
  // SETUP
  const Frame world_frame{Frame::new_frame()};
  const Frame curve_frame{Frame::new_frame()};
  const TCurve test_curve{
      curves::testing::make_circle_curve(curve_frame, world_frame)};

  const time::Timestamp time{10000ms};
  const time::Duration pose_period{100ms};

  // ACTION
  ::foxglove::PosesInFrame poses;
  poses_in_frame_from_t_curve(
      test_curve,
      test_curve.reference_frame().id().to_string(),
      time,
      pose_period,
      &poses);

  // VERIFICATION
  const time::SecsAndNanos expected_secs_and_nanos{
      time::to_seconds_and_nanos(time.time_since_epoch())};
  EXPECT_EQ(poses.timestamp().seconds(), expected_secs_and_nanos.secs);
  EXPECT_EQ(poses.timestamp().nanos(), expected_secs_and_nanos.nanos);
  EXPECT_EQ(world_frame.id(), UUID{poses.frame_id()});

  const time::Timestamp start_time{time::as_duration(test_curve.start_time())};
  const time::Timestamp end_time{time::as_duration(test_curve.end_time())};
  expect_poses_correct(
      test_curve,
      start_time,
      end_time,
      time,
      pose_period,
      poses);
}

}  // namespace resim::visualization::curve
