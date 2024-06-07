// Copyright 2024 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/visualization/log/visualize_actor_states.hh"

#include <fmt/core.h>
#include <foxglove/FrameTransform.pb.h>
#include <foxglove/SceneUpdate.pb.h>
#include <google/protobuf/util/message_differencer.h>
#include <gtest/gtest.h>

#include <mcap/reader.hpp>
#include <sstream>
#include <string>
#include <variant>
#include <vector>

#include "resim/actor/state/observable_state.hh"
#include "resim/actor/state/proto/observable_state.pb.h"
#include "resim/actor/state/proto/observable_state_to_proto.hh"
#include "resim/actor/state/rigid_body_state.hh"
#include "resim/assert/assert.hh"
#include "resim/curves/two_jet.hh"
#include "resim/curves/two_jet_test_helpers.hh"
#include "resim/experiences/experience.hh"
#include "resim/experiences/proto/experience.pb.h"
#include "resim/experiences/proto/experience_to_proto.hh"
#include "resim/experiences/proto/experiences_test_helpers.hh"
#include "resim/geometry/wireframe.hh"
#include "resim/simulator/standard_frames.hh"
#include "resim/time/proto/time_to_proto.hh"
#include "resim/time/timestamp.hh"
#include "resim/transforms/frame.hh"
#include "resim/transforms/se3.hh"
#include "resim/transforms/so3.hh"
#include "resim/utils/mcap_logger.hh"
#include "resim/visualization/foxglove/frame_transform_to_foxglove.hh"
#include "resim/visualization/foxglove/wireframe_to_foxglove.hh"
#include "resim/visualization/log/test_helpers.hh"
#include "resim/visualization/log/visit_topic.hh"

namespace resim::visualization::log {

namespace {

constexpr time::Timestamp PRE_SPAWN_TIME{time::as_duration(0.)};
constexpr time::Timestamp SPAWN_TIME{time::as_duration(1.)};
constexpr time::Timestamp ACTIVE_TIME{time::as_duration(2.)};
constexpr time::Timestamp DESPAWN_TIME{time::as_duration(3.)};
constexpr time::Timestamp AFTER_DESPAWN_TIME{time::as_duration(4.)};

const experiences::Experience experience{test_experience()};

std::vector<actor::state::ObservableState> make_actor_states() {
  curves::TwoJetTestHelper<curves::TwoJetR<transforms::SE3>> two_jet_helper;
  REASSERT(
      experience.dynamic_behavior.actors.size() == 1U,
      "The test experience should only have one actor!");
  auto actor_id = experience.dynamic_behavior.actors.front().id;

  const auto random_rigid_body_state = [&]() {
    actor::state::RigidBodyState<transforms::SE3> result{
        two_jet_helper.make_test_two_jet()};

    auto ref_from_body = result.ref_from_body();
    ref_from_body.set_frames(
        simulator::SCENE_FRAME,
        transforms::Frame<transforms::SE3::DIMS>(actor_id));
    result.set_ref_from_body(ref_from_body);

    return result;
  };

  return {
      actor::state::ObservableState{
          .id = actor_id,
          .is_spawned = false,
          .time_of_validity = PRE_SPAWN_TIME,
          .state = random_rigid_body_state(),
      },
      actor::state::ObservableState{
          .id = actor_id,
          .is_spawned = true,
          .time_of_validity = SPAWN_TIME,
          .state = random_rigid_body_state(),
      },
      actor::state::ObservableState{
          .id = actor_id,
          .is_spawned = true,
          .time_of_validity = ACTIVE_TIME,
          .state = random_rigid_body_state(),
      },
      actor::state::ObservableState{
          .id = actor_id,
          .is_spawned = false,
          .time_of_validity = DESPAWN_TIME,
          .state = random_rigid_body_state(),
      },
      actor::state::ObservableState{
          .id = actor_id,
          .is_spawned = false,
          .time_of_validity = AFTER_DESPAWN_TIME,
          .state = random_rigid_body_state(),
      },
  };
}
const auto actor_states{make_actor_states()};

std::string make_input_log_with_actor_states() {
  experiences::proto::Experience experience_msg;
  pack(experience, &experience_msg);

  std::ostringstream sstream;
  // Scope so that the logger flushes to the string stream when it's destroyed.
  {
    McapLogger logger{sstream};

    // Add the experience
    logger.add_proto_channel<experiences::proto::Experience>("/experience");
    constexpr time::Timestamp TIME;
    logger.log_proto("/experience", TIME, experience_msg);

    // Add some actor states
    logger.add_proto_channel<actor::state::proto::ObservableStates>(
        "/actor_states");

    const auto log_actor_state =
        [&](const actor::state::ObservableState& state) {
          std::vector<actor::state::ObservableState> states = {state};
          actor::state::proto::ObservableStates states_msg;
          pack(states, &states_msg);
          logger.log_proto("/actor_states", state.time_of_validity, states_msg);
        };

    for (const auto& actor_state : actor_states) {
      log_actor_state(actor_state);
    }
  }
  return sstream.str();
}

void validate_geometry_updates(
    const std::vector<::foxglove::SceneUpdate>& geometry_updates) {
  ASSERT_EQ(geometry_updates.size(), actor_states.size());

  // Before Spawning
  {
    EXPECT_EQ(geometry_updates.at(0).entities_size(), 0);
    EXPECT_EQ(geometry_updates.at(0).deletions_size(), 0);
  }
  // Spawning
  {
    ASSERT_EQ(geometry_updates.at(1).entities_size(), 1);
    EXPECT_EQ(geometry_updates.at(2).deletions_size(), 0);
    const auto& entity = geometry_updates.at(1).entities(0);
    EXPECT_EQ(time::proto::unpack(entity.timestamp()), SPAWN_TIME);
    EXPECT_EQ(
        entity.frame_id(),
        actor_states.at(1).state.ref_from_body().from().id().to_string());
    EXPECT_EQ(entity.id(), actor_states.at(1).id.to_string());

    ASSERT_EQ(experience.geometries.size(), 1U);
    const auto& geometry = experience.geometries.cbegin()->second.model;
    ASSERT_TRUE(std::holds_alternative<geometry::Wireframe>(geometry));

    ::foxglove::LinePrimitive expected_primitive;
    foxglove::pack_into_foxglove(
        std::get<geometry::Wireframe>(geometry),
        &expected_primitive);
    ASSERT_EQ(entity.lines_size(), 1U);
    EXPECT_TRUE(::google::protobuf::util::MessageDifferencer::Equals(
        expected_primitive,
        entity.lines(0)));
  }
  // Active
  {
    EXPECT_EQ(geometry_updates.at(2).entities_size(), 0);
    EXPECT_EQ(geometry_updates.at(2).deletions_size(), 0);
  }
  // Despawn
  {
    EXPECT_EQ(geometry_updates.at(3).entities_size(), 0);
    ASSERT_EQ(geometry_updates.at(3).deletions_size(), 1);
    const auto& deletion = geometry_updates.at(3).deletions(0);
    EXPECT_EQ(time::proto::unpack(deletion.timestamp()), DESPAWN_TIME);
    EXPECT_EQ(deletion.type(), ::foxglove::SceneEntityDeletion::MATCHING_ID);
    EXPECT_EQ(deletion.id(), actor_states.at(3).id.to_string());
  }
  // After Despawn
  {
    EXPECT_EQ(geometry_updates.at(4).entities_size(), 0);
    EXPECT_EQ(geometry_updates.at(4).deletions_size(), 0);
  }
}

void validate_frame_transforms(
    const std::vector<::foxglove::FrameTransform>& frame_transforms) {
  int spawned_state_idx = 0;
  for (const auto& state : actor_states) {
    if (state.is_spawned) {
      ASSERT_LT(spawned_state_idx, frame_transforms.size());

      const transforms::SE3& scene_from_actor{state.state.ref_from_body()};

      const ::foxglove::FrameTransform& transform =
          frame_transforms.at(spawned_state_idx);
      const transforms::SE3 packed_scene_from_actor{
          transforms::SO3{Eigen::Quaterniond{
              transform.rotation().w(),
              transform.rotation().x(),
              transform.rotation().y(),
              transform.rotation().z()}},
          {transform.translation().x(),
           transform.translation().y(),
           transform.translation().z()}};

      EXPECT_EQ(transform.parent_frame_id(), simulator::SCENE_FRAME_NAME);
      EXPECT_EQ(
          transform.child_frame_id(),
          scene_from_actor.from().id().to_string());

      EXPECT_TRUE(
          packed_scene_from_actor.is_approx_transform(scene_from_actor));

      ++spawned_state_idx;
    }
  }
  EXPECT_EQ(spawned_state_idx, frame_transforms.size());
}

}  // namespace

TEST(VisualizeActorStatesTest, TestVisualizeActorStates) {
  // SETUP
  std::istringstream input_log{make_input_log_with_actor_states()};

  mcap::McapReader reader;
  StreamReader readable{input_log};
  EXPECT_TRUE(reader.open(readable).ok());

  std::ostringstream out_log_stream;
  {
    McapLogger logger{out_log_stream};

    // ACTION
    visualize_actor_states(experience, InOut{reader}, InOut{logger});
  }

  // VERIFICATION
  std::stringstream output_log{out_log_stream.str()};
  mcap::McapReader output_reader;
  StreamReader out_readable{output_log};
  EXPECT_TRUE(output_reader.open(out_readable).ok());

  std::vector<::foxglove::SceneUpdate> geometry_updates;
  visit_topic(
      "/geometries",
      InOut{output_reader},
      [&](const mcap::MessageView& view) {
        ASSERT_EQ(
            view.schema->name,
            ::foxglove::SceneUpdate::GetDescriptor()->full_name());
        ::foxglove::SceneUpdate update;
        ASSERT_TRUE(
            update.ParseFromArray(view.message.data, view.message.dataSize));
        geometry_updates.push_back(update);
      });
  validate_geometry_updates(geometry_updates);

  std::vector<::foxglove::FrameTransform> frame_transforms;
  auto actor_id = experience.dynamic_behavior.actors.front().id;
  const std::string frame_transform_topic = fmt::format(
      "/transforms/{}_from_{}",
      simulator::SCENE_FRAME_NAME,
      actor_id.to_string());

  visit_topic(
      frame_transform_topic,
      InOut{output_reader},
      [&](const mcap::MessageView& view) {
        ASSERT_EQ(
            view.schema->name,
            ::foxglove::FrameTransform::GetDescriptor()->full_name());
        ::foxglove::FrameTransform transform;
        ASSERT_TRUE(
            transform.ParseFromArray(view.message.data, view.message.dataSize));
        frame_transforms.push_back(transform);
      });
  validate_frame_transforms(frame_transforms);
}

TEST(VisualizeActorStatesTest, TestVisualizeActorStatesNoGeometry) {
  // SETUP
  std::istringstream input_log{make_input_log_with_actor_states()};

  mcap::McapReader reader;
  StreamReader readable{input_log};
  EXPECT_TRUE(reader.open(readable).ok());

  auto experience_without_geometry = experience;
  experience_without_geometry.dynamic_behavior.actors.front()
      .geometries.clear();

  std::ostringstream out_log_stream;
  {
    McapLogger logger{out_log_stream};

    // ACTION
    visualize_actor_states(
        experience_without_geometry,
        InOut{reader},
        InOut{logger});
  }

  // VERIFICATION
  std::stringstream output_log{out_log_stream.str()};
  mcap::McapReader output_reader;
  StreamReader out_readable{output_log};
  EXPECT_TRUE(output_reader.open(out_readable).ok());

  visit_topic(
      "/geometries",
      InOut{output_reader},
      [&](const mcap::MessageView& view) {
        ASSERT_EQ(
            view.schema->name,
            ::foxglove::SceneUpdate::GetDescriptor()->full_name());
        ::foxglove::SceneUpdate update;
        ASSERT_TRUE(
            update.ParseFromArray(view.message.data, view.message.dataSize));

        EXPECT_EQ(update.entities_size(), 0U);
        EXPECT_EQ(update.deletions_size(), 0U);
      });

  std::vector<::foxglove::FrameTransform> frame_transforms;
  auto actor_id = experience.dynamic_behavior.actors.front().id;
  const std::string frame_transform_topic = fmt::format(
      "/transforms/{}_from_{}",
      simulator::SCENE_FRAME_NAME,
      actor_id.to_string());

  visit_topic(
      frame_transform_topic,
      InOut{output_reader},
      [&](const mcap::MessageView& view) {
        ASSERT_EQ(
            view.schema->name,
            ::foxglove::FrameTransform::GetDescriptor()->full_name());
        ::foxglove::FrameTransform transform;
        ASSERT_TRUE(
            transform.ParseFromArray(view.message.data, view.message.dataSize));
        frame_transforms.push_back(transform);
      });
  validate_frame_transforms(frame_transforms);
}

}  // namespace resim::visualization::log
