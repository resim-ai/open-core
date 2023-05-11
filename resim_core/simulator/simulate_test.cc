
#include "resim_core/simulator/simulate.hh"

#include <fmt/core.h>
#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <mcap/reader.hpp>
#include <ratio>
#include <vector>

#include "resim_core/actor/actor_id.hh"
#include "resim_core/actor/state/trajectory.hh"
#include "resim_core/assert/assert.hh"
#include "resim_core/curves/test_helpers.hh"
#include "resim_core/experiences/actor.hh"
#include "resim_core/experiences/completion_criteria.hh"
#include "resim_core/experiences/dynamic_behavior.hh"
#include "resim_core/experiences/experience.hh"
#include "resim_core/experiences/storyboard.hh"
#include "resim_core/simulator/standard_frames.hh"
#include "resim_core/testing/test_directory.hh"
#include "resim_core/time/timestamp.hh"
#include "resim_core/transforms/frame.hh"

namespace resim::simulator {

namespace {

constexpr time::Timestamp START_TIME;
constexpr time::Duration DURATION{std::chrono::seconds(30)};

const actor::ActorId actor_id{actor::ActorId::new_uuid()};

const transforms::Frame<3> actor_frame{transforms::Frame<3>::new_frame()};

const actor::state::Trajectory actor_trajectory{
    curves::testing::make_circle_curve(actor_frame, SCENE_FRAME),
    START_TIME};

const experiences::Experience test_experience{
    .dynamic_behavior =
        experiences::DynamicBehavior{
            .actors =
                std::vector<experiences::Actor>{
                    {
                        .id = actor_id,
                        .name = "test_actor",
                        .actor_type = experiences::ActorType::SYSTEM_UNDER_TEST,
                    },
                },
            .storyboard =
                experiences::Storyboard{
                    .movement_models =
                        std::vector<experiences::MovementModel>{
                            {
                                .actor_reference = actor_id,
                                .model = actor_trajectory,
                            },
                        }},
            .completion_criteria =
                experiences::CompletionCriteria{
                    .time_limit = DURATION,
                },
        },
};
}  // namespace

// NOLINTBEGIN(readability-function-cognitive-complexity)
TEST(SimulateTest, TestRunSim) {
  // SETUP
  testing::TestDirectoryRAII test_directory;

  const std::filesystem::path mcap_path{test_directory.test_file_path("mcap")};

  // ACTION
  simulate(test_experience, mcap_path);

  // VERIFICATION
  ASSERT_TRUE(std::filesystem::exists(mcap_path));
  mcap::McapReader reader;
  ASSERT_EQ(reader.open(mcap_path.c_str()).code, mcap::StatusCode::Success);

  ASSERT_TRUE(reader.readSummary(mcap::ReadSummaryMethod::NoFallbackScan).ok());
  const auto &channels = reader.channels();
  ASSERT_EQ(channels.size(), 2U);

  time::Timestamp max_time{time::Duration::min()};
  time::Timestamp min_time{time::Duration::max()};

  std::unordered_map<std::string, int> message_counts;
  for (const mcap::MessageView &view : reader.readMessages()) {
    const time::Timestamp time{std::chrono::nanoseconds(view.message.logTime)};
    max_time = std::max(max_time, time);
    min_time = std::min(min_time, time);
    ++message_counts[view.channel->topic];
  }
  EXPECT_EQ(min_time, START_TIME);
  EXPECT_EQ(max_time, START_TIME + DURATION);

  const std::string transform_topic{fmt::format(
      "/transforms/{}_from_{}",
      SCENE_FRAME_NAME,
      actor_trajectory.body_frame().id().to_string())};
  ASSERT_TRUE(message_counts.contains(transform_topic));
  constexpr time::Duration DT{std::chrono::milliseconds(10)};
  EXPECT_EQ(
      message_counts.at(transform_topic),
      actor_trajectory.time_duration() / DT + 1U);  // Fence post

  constexpr auto GEOMETRIES_TOPIC = "/geometries";
  ASSERT_TRUE(message_counts.contains(GEOMETRIES_TOPIC));
  EXPECT_EQ(message_counts.at(GEOMETRIES_TOPIC), DURATION / DT + 1U);
}

TEST(SimulateTest, TestRunSimNegativeTimeFails) {
  // SETUP
  testing::TestDirectoryRAII test_directory;

  const std::filesystem::path mcap_path{test_directory.test_file_path("mcap")};

  for (const int time_ns : {0, -1, -5}) {
    auto negative_time_experience = test_experience;
    negative_time_experience.dynamic_behavior.completion_criteria.time_limit =
        std::chrono::nanoseconds(time_ns);

    // ACTION / VERIFICATION
    EXPECT_THROW(
        simulate(negative_time_experience, mcap_path),
        AssertException);
  }
}
// NOLINTEND(readability-function-cognitive-complexity)

}  // namespace resim::simulator