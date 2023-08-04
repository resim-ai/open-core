// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.


#include "resim/simulator/simulate.hh"

#include <fmt/core.h>
#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <mcap/reader.hpp>
#include <ratio>
#include <vector>

#include "resim/actor/actor_id.hh"
#include "resim/actor/state/trajectory.hh"
#include "resim/assert/assert.hh"
#include "resim/curves/test_helpers.hh"
#include "resim/experiences/actor.hh"
#include "resim/experiences/completion_criteria.hh"
#include "resim/experiences/dynamic_behavior.hh"
#include "resim/experiences/experience.hh"
#include "resim/experiences/proto/experience.pb.h"
#include "resim/experiences/proto/experience_to_proto.hh"
#include "resim/experiences/storyboard.hh"
#include "resim/simulator/standard_frames.hh"
#include "resim/testing/test_directory.hh"
#include "resim/time/timestamp.hh"
#include "resim/transforms/frame.hh"

namespace resim::simulator {

namespace {

constexpr time::Timestamp START_TIME{std::chrono::seconds(0)};
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

  // TODO(tknowles): Currently this includes the single metric_min_distance
  // channel, which will likely change.
  ASSERT_EQ(channels.size(), 3U);

  for (const auto &[a, b] : channels) {
    std::cout << b->topic << std::endl;
  }

  time::Timestamp max_time{time::Duration::min()};
  time::Timestamp min_time{time::Duration::max()};

  constexpr auto EXPERIENCE_TOPIC = "/experience";
  std::unordered_map<std::string, int> message_counts;
  for (const mcap::MessageView &view : reader.readMessages()) {
    const time::Timestamp time{std::chrono::nanoseconds(view.message.logTime)};
    max_time = std::max(max_time, time);
    min_time = std::min(min_time, time);
    ++message_counts[view.channel->topic];

    if (view.channel->topic == EXPERIENCE_TOPIC) {
      experiences::proto::Experience experience_msg;
      pack(test_experience, &experience_msg);
      const std::string expected{experience_msg.SerializeAsString()};
      ASSERT_EQ(expected.size(), view.message.dataSize);
      EXPECT_EQ(
          std::memcmp(
              expected.data(),
              view.message.data,
              view.message.dataSize),
          0);
    }
  }
  EXPECT_EQ(min_time, START_TIME);
  EXPECT_EQ(max_time, START_TIME + DURATION);

  constexpr time::Duration DT{std::chrono::milliseconds(10)};

  constexpr auto ACTOR_STATES_TOPIC = "/actor_states";
  ASSERT_TRUE(message_counts.contains(ACTOR_STATES_TOPIC));
  EXPECT_EQ(message_counts.at(ACTOR_STATES_TOPIC), DURATION / DT + 1U);

  ASSERT_TRUE(message_counts.contains(EXPERIENCE_TOPIC));
  EXPECT_EQ(message_counts.at(EXPERIENCE_TOPIC), 1U);
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
