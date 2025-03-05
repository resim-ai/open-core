
#include "resim/visualization/curve/visualize_t_curve.hh"

#include <foxglove/LinePrimitive.pb.h>
#include <foxglove/PosesInFrame.pb.h>
#include <foxglove/SceneEntity.pb.h>
#include <foxglove/SceneUpdate.pb.h>
#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <filesystem>
#include <map>
#include <mcap/mcap.hpp>
#include <string>
#include <unordered_set>
#include <vector>

#include "resim/curves/t_curve.hh"
#include "resim/testing/random_matrix.hh"
#include "resim/testing/test_directory.hh"
#include "resim/transforms/frame.hh"
#include "resim/transforms/se3.hh"
#include "resim/utils/inout.hh"
#include "resim/utils/uuid.hh"

namespace resim::visualization::curve {
namespace {

using transforms::SE3;
using Frame = transforms::Frame<SE3::DIMS>;
using TwoJetL = curves::TwoJetL<SE3>;
using TCurve = curves::TCurve<SE3>;
using TangentVector = SE3::TangentVector;

const Frame reference_frame{UUID{"6466f72f-f317-43a1-9a14-1bd773d4bc4c"}};
constexpr uint64_t SEED = 143U;
constexpr unsigned NUM_CONTROLS = 10;
constexpr unsigned CURVE_COUNT = 4;
const std::array<std::string, CURVE_COUNT> curve_names{
    "Curve_00",
    "Curve_01",
    "Curve_02",
    "Curve_03"};
// Note the colors below were chosen especially because all their rgb values
// are non zero. Consider this before changing them.
const std::map<std::string, Color> colormap{
    {curve_names.at(0), colors::HOTPINK},
    {curve_names.at(1), colors::BISQUE},
    {curve_names.at(2), colors::THISTLE},
    {curve_names.at(3), colors::ORCHID}};

// This helper makes a randomly generated TCurve that goes in a cycle.
TCurve make_t_curve() {
  using Control = TCurve::Control;
  std::mt19937 rng{SEED};
  const Frame curve_frame{Frame::new_frame()};
  std::vector<Control> controls;
  for (int ii = 0.; ii < NUM_CONTROLS; ++ii) {
    const double t = ii;
    controls.push_back(Control{
        .time = t,
        .point =
            TwoJetL{
                SE3::exp(
                    testing::random_vector<TangentVector>(rng),
                    curve_frame,
                    reference_frame),
                testing::random_vector<TangentVector>(rng),
                testing::random_vector<TangentVector>(rng),
            },

    });
  }
  // Make the TCurve a cycle:
  controls.push_back(controls.front());
  controls.back().time = static_cast<double>(NUM_CONTROLS);
  return TCurve{controls};
}

// NOLINTBEGIN(readability-function-cognitive-complexity)
void evaluate_line_entity(const ::foxglove::SceneEntity &entity) {
  // Check that the frame id of the line matches the reference frame.
  EXPECT_EQ(entity.frame_id(), reference_frame.id().to_string());
  // Expecting one line per entity.
  ASSERT_EQ(entity.lines().size(), 1);
  // Compare the color of the line with the expected color.
  const auto line_color = entity.lines(0).color();
  const auto expected_color = colormap.at(entity.id());
  EXPECT_DOUBLE_EQ(line_color.r(), expected_color.r);
  EXPECT_DOUBLE_EQ(line_color.g(), expected_color.g);
  EXPECT_DOUBLE_EQ(line_color.b(), expected_color.b);
  EXPECT_DOUBLE_EQ(line_color.a(), expected_color.a);
  // Cover the everything is equal and zero edge case.
  EXPECT_NE(line_color.r(), 0.);
  EXPECT_NE(line_color.g(), 0.);
  EXPECT_NE(line_color.b(), 0.);
}
// NOLINTEND(readability-function-cognitive-complexity)

void evaluate_scene_update_message(const mcap::MessageView &view) {
  // Extract the SceneUpdate message
  ::foxglove::SceneUpdate update;
  ASSERT_TRUE(update.ParseFromArray(
      static_cast<const void *>(view.message.data),
      view.message.dataSize));
  // Number of entities should match the number of curves.
  EXPECT_EQ(CURVE_COUNT, update.entities().size());
  for (const auto &entity : update.entities()) {
    evaluate_line_entity(entity);
  }
}

void evaluate_pose_in_frame_message(const mcap::MessageView &view) {
  // Extract the PosesInFrame message.
  ::foxglove::PosesInFrame poses;
  ASSERT_TRUE(poses.ParseFromArray(
      static_cast<const void *>(view.message.data),
      view.message.dataSize));
  // Verify the expected number of poses.
  EXPECT_EQ(CURVE_COUNT * NUM_CONTROLS, poses.poses().size());
}
}  // namespace

// NOLINTBEGIN(readability-function-cognitive-complexity)
TEST(VisualizeTCurveTest, TestMcapMessageCountsSingle) {
  // SETUP
  const testing::TestDirectoryRAII test_directory;
  const std::filesystem::path test_mcap{test_directory.test_file_path("mcap")};

  auto test_curve = make_t_curve();
  CurveVisualizationOptions options;
  const std::string SCENE_UPDATE_TOPIC = "/t_curve/line";
  const std::string POSES_IN_FRAME_TOPIC = "/t_curve/escalator_frames";

  // ACTION
  {
    McapLogger logger{test_mcap};
    visualize_t_curve(
        test_curve,
        options,
        SCENE_UPDATE_TOPIC,
        POSES_IN_FRAME_TOPIC,
        InOut{logger});
  }

  // VERIFICATION
  mcap::McapReader reader;
  ASSERT_TRUE(reader.open(test_mcap.string()).ok());

  // Check that we have the right number of channels
  ASSERT_TRUE(reader.readSummary(mcap::ReadSummaryMethod::NoFallbackScan).ok());
  const auto &channels = reader.channels();
  constexpr int NUM_CHANNELS = 2;
  ASSERT_EQ(channels.size(), NUM_CHANNELS);

  // Check that we have the expected number of messages published per channel
  int scene_count = 0;
  int poses_count = 0;
  for (const mcap::MessageView &view : reader.readMessages()) {
    if (view.channel->topic == SCENE_UPDATE_TOPIC) {
      ++scene_count;
    } else if (view.channel->topic == POSES_IN_FRAME_TOPIC) {
      // Note because we pass a single curve in this example we can assume
      // that this curve's start time is the smallest timestamp.
      const time::Duration expected_time{
          time::as_duration(test_curve.start_time()) +
          poses_count * options.publish_rate};
      EXPECT_EQ(view.message.logTime, expected_time.count());
      EXPECT_EQ(view.message.publishTime, expected_time.count());
      ++poses_count;
    } else {
      ASSERT_TRUE(false) << "Unexpected topic found";
    }
  }
  // The Scene is sent only once.
  EXPECT_EQ(scene_count, 1);
  // Poses are sent once per publish cycle.
  const uint64_t expected_poses_count =
      (options.pose_period.count() / options.publish_rate.count()) + 1;
  EXPECT_EQ(poses_count, expected_poses_count);
  // Clean up the mcap
  reader.close();
}
// NOLINTEND(readability-function-cognitive-complexity)

// NOLINTBEGIN(readability-function-cognitive-complexity)
TEST(VisualizeTCurveTest, TestMcapMessageContentsMultiple) {
  // SETUP
  // We plan to make four curves with specific names and colors.
  const testing::TestDirectoryRAII test_directory;
  const std::filesystem::path test_mcap{test_directory.test_file_path("mcap")};

  CurveVisualizationOptions options;
  const std::string SCENE_UPDATE_TOPIC = "/t_curve/line";
  const std::string POSES_IN_FRAME_TOPIC = "/t_curve/escalator_frames";

  // ACTION
  {
    McapLogger logger{test_mcap};
    MultiTCurveVisualizer viz(
        options,
        SCENE_UPDATE_TOPIC,
        POSES_IN_FRAME_TOPIC,
        InOut{logger});
    for (unsigned i = 0; i < CURVE_COUNT; ++i) {
      SingleCurveOptions curve_options;
      curve_options.unique_name = curve_names.at(i);
      curve_options.line_options.color = colormap.at(curve_options.unique_name);
      viz.add_curve(make_t_curve(), curve_options);
    }
  }

  // VERIFICATION
  mcap::McapReader reader;
  ASSERT_TRUE(reader.open(test_mcap.string()).ok());

  // Check the contents of the messages and verify names, colors, reference
  // frames and expected number of poses.
  for (const mcap::MessageView &view : reader.readMessages()) {
    if (view.channel->topic == SCENE_UPDATE_TOPIC) {
      evaluate_scene_update_message(view);
    } else if (view.channel->topic == POSES_IN_FRAME_TOPIC) {
      evaluate_pose_in_frame_message(view);
    } else {
      ASSERT_TRUE(false) << "Unexpected topic found";
    }
  }
  // Clean up the mcap
  reader.close();
}
// NOLINTEND(readability-function-cognitive-complexity)

TEST(VisualizeTCurveTest, TestNoEscalatorFrames) {
  // SETUP
  // We plan to make four curves with specific names and colors.
  // But no escalator frames.
  const testing::TestDirectoryRAII test_directory;
  const std::filesystem::path test_mcap{test_directory.test_file_path("mcap")};

  CurveVisualizationOptions options;
  const std::string SCENE_UPDATE_TOPIC = "/t_curve/line";
  const std::string POSES_IN_FRAME_TOPIC = "/t_curve/escalator_frames";

  // ACTION
  {
    McapLogger logger{test_mcap};
    MultiTCurveVisualizer viz(
        options,
        SCENE_UPDATE_TOPIC,
        POSES_IN_FRAME_TOPIC,
        InOut{logger});
    for (unsigned i = 0; i < CURVE_COUNT; ++i) {
      SingleCurveOptions curve_options;
      curve_options.unique_name = curve_names.at(i);
      curve_options.line_options.color = colormap.at(curve_options.unique_name);
      curve_options.include_escalator_frames = false;
      viz.add_curve(make_t_curve(), curve_options);
    }
  }

  // VERIFICATION
  mcap::McapReader reader;
  ASSERT_TRUE(reader.open(test_mcap.string()).ok());

  // Check the contents of the messages and verify names, colors, reference
  // frames and expected number of poses.
  for (const mcap::MessageView &view : reader.readMessages()) {
    // only scene update messages should exist
    EXPECT_TRUE(view.channel->topic == SCENE_UPDATE_TOPIC);
  }
  // Clean up the mcap
  reader.close();
}

// NOLINTBEGIN(readability-function-cognitive-complexity)
TEST(VisualizeTCurveTest, TestAutoUniqueNames) {
  // SETUP
  const testing::TestDirectoryRAII test_directory;
  const std::filesystem::path test_mcap{test_directory.test_file_path("mcap")};

  CurveVisualizationOptions options;
  const std::string SCENE_UPDATE_TOPIC = "/t_curve/line";
  const std::string POSES_IN_FRAME_TOPIC = "/t_curve/escalator_frames";

  // ACTION
  {
    McapLogger logger{test_mcap};
    MultiTCurveVisualizer viz(
        options,
        SCENE_UPDATE_TOPIC,
        POSES_IN_FRAME_TOPIC,
        InOut{logger});
    for (unsigned i = 0; i < CURVE_COUNT; ++i) {
      // Don't Specify any options.
      viz.add_curve(make_t_curve());
    }
  }

  // VERIFICATION
  mcap::McapReader reader;
  ASSERT_TRUE(reader.open(test_mcap.string()).ok());

  // Count the number of unique ids showing up in the messages. They should
  // be exactly equal to the number of curves.
  for (const mcap::MessageView &view : reader.readMessages()) {
    std::unordered_set<std::string> ids;
    if (view.channel->topic == SCENE_UPDATE_TOPIC) {
      // Extract the SceneUpdate message
      ::foxglove::SceneUpdate update;
      ASSERT_TRUE(update.ParseFromArray(
          static_cast<const void *>(view.message.data),
          view.message.dataSize));
      // Number of entities should match the number of curves.
      EXPECT_EQ(CURVE_COUNT, update.entities().size());
      for (const auto &entity : update.entities()) {
        ids.insert(entity.id());
      }
      // We expect ids to have accumulated CURVE_COUNT unique ids.
      EXPECT_EQ(CURVE_COUNT, ids.size());
    }
  }
  // Clean up the mcap
  reader.close();
}
// NOLINTEND(readability-function-cognitive-complexity)

}  // namespace resim::visualization::curve
