// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/geometry/proto/wireframe_to_proto.hh"

#include <gtest/gtest.h>

#include <random>

#include "resim/assert/assert.hh"
#include "resim/geometry/proto/wireframe.pb.h"
#include "resim/geometry/wireframe.hh"
#include "resim/testing/random_matrix.hh"

namespace resim::geometry {

// Helper to compare the points in a Wireframe to those in the corresponding
// proto message.
void expect_points_match(
    const Wireframe &wireframe,
    const proto::Wireframe &message) {
  ASSERT_EQ(wireframe.points().size(), message.points_size());
  for (int ii = 0; ii < wireframe.points().size(); ++ii) {
    const Wireframe::Point &point = wireframe.points().at(ii);
    const proto::Wireframe::Point &point_msg = message.points(ii);
    ASSERT_EQ(point_msg.values_size(), 3);
    for (int jj = 0; jj < 3; ++jj) {
      EXPECT_EQ(point(jj), point_msg.values(jj));
    }
  }
}

// Helper to compare the edges in a Wireframe to those in the corresponding
// proto message.
void expect_edges_match(
    const Wireframe &wireframe,
    const proto::Wireframe &message) {
  ASSERT_EQ(wireframe.edges().size(), message.edges_size());
  for (int ii = 0; ii < wireframe.edges().size(); ++ii) {
    const Wireframe::Edge &edge = wireframe.edges().at(ii);
    const proto::Wireframe::Edge &edge_msg = message.edges(ii);
    EXPECT_EQ(edge.at(0), edge_msg.start());
    EXPECT_EQ(edge.at(1), edge_msg.end());
  }
}

TEST(WireframeToProtoTest, TestPack) {
  // SETUP
  constexpr unsigned SEED = 82U;
  std::mt19937 rng{SEED};

  Wireframe test_wireframe;
  test_wireframe.add_point(testing::random_vector<Eigen::Vector3d>(rng));
  test_wireframe.add_point(testing::random_vector<Eigen::Vector3d>(rng));
  test_wireframe.add_edge({0U, 1U});

  // ACTION
  proto::Wireframe message;
  pack(test_wireframe, &message);

  // VERIFICATION
  expect_points_match(test_wireframe, message);
  expect_edges_match(test_wireframe, message);
}

TEST(WireframeToProtoTest, TestPackInvalid) {
  // SETUP
  constexpr unsigned SEED = 82U;
  std::mt19937 rng{SEED};

  Wireframe test_wireframe;
  test_wireframe.add_point(testing::random_vector<Eigen::Vector3d>(rng));
  test_wireframe.add_point(testing::random_vector<Eigen::Vector3d>(rng));
  test_wireframe.add_edge({0U, 2U});

  // ACTION / VERIFICATION
  proto::Wireframe message;
  EXPECT_THROW(pack(test_wireframe, &message), AssertException);
}

TEST(WireframeToProtoTest, TestUnpack) {
  // SETUP
  constexpr unsigned SEED = 82U;
  std::mt19937 rng{SEED};

  Wireframe test_wireframe;
  test_wireframe.add_point(testing::random_vector<Eigen::Vector3d>(rng));
  test_wireframe.add_point(testing::random_vector<Eigen::Vector3d>(rng));
  test_wireframe.add_edge({0U, 1U});

  proto::Wireframe message;
  pack(test_wireframe, &message);

  // ACTION
  const Wireframe unpacked{unpack(message)};

  // VERIFICATION
  EXPECT_EQ(test_wireframe.points(), unpacked.points());
  EXPECT_EQ(test_wireframe.edges(), unpacked.edges());
}

TEST(WireframeToProtoTest, TestUnpackInvalid) {
  // SETUP
  constexpr unsigned SEED = 82U;
  std::mt19937 rng{SEED};

  Wireframe test_wireframe;
  test_wireframe.add_point(testing::random_vector<Eigen::Vector3d>(rng));
  test_wireframe.add_point(testing::random_vector<Eigen::Vector3d>(rng));
  test_wireframe.add_edge({0U, 1U});

  proto::Wireframe message;
  pack(test_wireframe, &message);

  // Make it invalid
  message.mutable_edges(0)->set_end(2U);

  // ACTION / VERIFICATION
  EXPECT_THROW(unpack(message), AssertException);
}

}  // namespace resim::geometry
