
#include <foxglove/Point3.pb.h>
#include <foxglove/Pose.pb.h>
#include <gtest/gtest.h>

#include "resim/assert/assert.hh"
#include "resim/curves/t_curve.hh"
#include "resim/curves/test_helpers.hh"
#include "resim/time/sample_interval.hh"
#include "resim/transforms/se3.hh"
#include "resim/visualization/curve/esc_frames_from_t_curve.hh"
#include "resim/visualization/curve/poses_in_frame_from_t_curve.hh"
#include "resim/visualization/foxglove/pose_to_foxglove.hh"

namespace resim::visualization::curve {

using transforms::SE3;
using TCurve = curves::TCurve<SE3>;
using Frame = transforms::Frame<SE3::DIMS>;
using Vec3 = Eigen::Vector3d;

namespace {
struct TestTCurveVizOptions {
  static constexpr int DEFAULT_PUBLISH_RATE_MS = 100;
  static constexpr int DEFAULT_POSE_PERIOD_S = 10;

  // Publish rate for the line and escalator frame messages
  time::Duration publish_rate{
      std::chrono::milliseconds(DEFAULT_PUBLISH_RATE_MS)};

  // The time interval along the curve between adjacent escalator frames.
  time::Duration pose_period{std::chrono::seconds(DEFAULT_POSE_PERIOD_S)};
};

}  // namespace

TEST(EscFramesFromTCurve, TestSampleCircle) {
  // SETUP
  const Frame world_frame{Frame::new_frame()};
  const Frame curve_frame{Frame::new_frame()};
  const TCurve curve{
      curves::testing::make_circle_curve(curve_frame, world_frame)};

  // ACTION
  const TestTCurveVizOptions options;
  const auto &esc_frames = esc_frames_from_t_curve(
      curve,
      curve.reference_frame().id().to_string(),
      options.pose_period,
      options.publish_rate);

  // VERIFICATION
  for (const auto &[time, pose] : esc_frames) {
    const time::SecsAndNanos expected_secs_and_nanos{
        time::to_seconds_and_nanos(time.time_since_epoch())};
    EXPECT_EQ(pose.timestamp().seconds(), expected_secs_and_nanos.secs);
    EXPECT_EQ(pose.timestamp().nanos(), expected_secs_and_nanos.nanos);
    EXPECT_EQ(world_frame.id(), UUID{pose.frame_id()});

    ::foxglove::PosesInFrame test_pose;
    poses_in_frame_from_t_curve(
        curve,
        curve.reference_frame().id().to_string(),
        time,
        options.pose_period,
        &test_pose);
    EXPECT_EQ(test_pose.SerializeAsString(), pose.SerializeAsString());
  }
}

}  // namespace resim::visualization::curve
