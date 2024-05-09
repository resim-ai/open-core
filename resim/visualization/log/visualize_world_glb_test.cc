
#include "resim/visualization/log/visualize_world_glb.hh"

#include <foxglove/SceneUpdate.pb.h>
#include <gtest/gtest.h>

#include <random>
#include <sstream>
#include <string>

#include "resim/converter/fuzz_helpers.hh"
#include "resim/simulator/standard_frames.hh"
#include "resim/time/proto/time_to_proto.hh"
#include "resim/time/timestamp.hh"
#include "resim/utils/inout.hh"
#include "resim/utils/mcap_logger.hh"
#include "resim/utils/testing/mock_logger.hh"

namespace resim::visualization::log {

namespace {
void expect_pose_identity(const ::foxglove::Pose &pose) {
  EXPECT_EQ(pose.position().x(), 0.0);
  EXPECT_EQ(pose.position().y(), 0.0);
  EXPECT_EQ(pose.position().z(), 0.0);

  EXPECT_EQ(pose.orientation().w(), 1.0);
  EXPECT_EQ(pose.orientation().x(), 0.0);
  EXPECT_EQ(pose.orientation().y(), 0.0);
  EXPECT_EQ(pose.orientation().z(), 0.0);
}

}  // namespace

TEST(VisualizeWorldGLBTest, TestLogWorldGLB) {
  // SETUP
  constexpr size_t SEED = 39U;
  std::mt19937 rng{SEED};
  testing::MockLogger::ChannelToMessageMap channel_to_message_map;
  testing::MockLogger mock_logger{channel_to_message_map};

  // Make a mock glb with a random string as its contents.
  const auto mock_file = converter::random_element<std::string>(InOut{rng});
  std::istringstream mock_glb_stream{mock_file};

  constexpr auto CHANNEL_NAME = "xXx_world_glb_channel_xXx";
  auto time = converter::random_element<time::Timestamp>(InOut{rng});

  // ACTION
  visualize_world_glb(
      mock_glb_stream,
      time,
      CHANNEL_NAME,
      InOut{static_cast<LoggerInterface &>(mock_logger)});

  // VERIFICATION
  ASSERT_EQ(channel_to_message_map.size(), 1U);
  ASSERT_TRUE(channel_to_message_map.contains(CHANNEL_NAME));
  ASSERT_EQ(channel_to_message_map.at(CHANNEL_NAME).size(), 1U);

  const auto &message = channel_to_message_map.at(CHANNEL_NAME).front();

  EXPECT_EQ(message.time, time);

  ::foxglove::SceneUpdate update;

  ASSERT_TRUE(update.ParseFromString(message.message));
  ASSERT_EQ(update.entities_size(), 1U);
  EXPECT_EQ(update.deletions_size(), 0U);

  const auto &entity = update.entities(0);

  EXPECT_EQ(time::proto::unpack(entity.timestamp()), time);
  EXPECT_EQ(entity.id(), "world_geometry");
  EXPECT_EQ(entity.frame_id(), simulator::SCENE_FRAME_NAME);
  ASSERT_EQ(entity.models_size(), 1U);
  const auto &model = entity.models(0);
  expect_pose_identity(model.pose());
  EXPECT_EQ(model.scale().x(), 1.0);
  EXPECT_EQ(model.scale().y(), 1.0);
  EXPECT_EQ(model.scale().z(), 1.0);
  EXPECT_EQ(model.media_type(), "model/gltb-binary");
  EXPECT_EQ(model.data(), mock_file);
}

}  // namespace resim::visualization::log
