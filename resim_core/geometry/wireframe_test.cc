#include "resim_core/geometry/wireframe.hh"

#include <gtest/gtest.h>

#include <vector>

namespace resim::geometry {

class WireframeTest : public ::testing::Test {
 protected:
  const std::vector<Wireframe::Point> points_{
      {-0.25, -0.5, 0.0},
      {-0.25, 0.5, 0.0},
      {0.75, 0.0, 0.0},
      {0.0, 0.0, 0.1},
  };

  const std::vector<Wireframe::Edge> edges_{
      {0, 1},
      {1, 2},
      {2, 0},
      {0, 3},
      {1, 3},
      {2, 3},
  };
};

TEST_F(WireframeTest, TestConstruction) {
  // ACTION
  const std::size_t invalid_index = points_.size();
  const Wireframe good_wireframe{points_, edges_};
  const Wireframe invalid_index_wireframe{points_, {{invalid_index, 1}}};
  const Wireframe duplicate_containing_wireframe{points_, {{1, 1}}};

  // VERIFICATION
  EXPECT_TRUE(good_wireframe.is_valid());
  EXPECT_FALSE(invalid_index_wireframe.is_valid());
  EXPECT_FALSE(duplicate_containing_wireframe.is_valid());
}

TEST_F(WireframeTest, TestGetters) {
  // SETUP
  const Wireframe wireframe{points_, edges_};
  // ACTION / VERIFICATION
  EXPECT_EQ(wireframe.points(), points_);
  EXPECT_EQ(wireframe.edges(), edges_);
}

TEST_F(WireframeTest, TestAdders) {
  // SETUP
  Wireframe wireframe;

  // ACTION
  for (const auto &edge : edges_) {
    wireframe.add_edge(edge);
  }

  // VERIFICATION
  // Should be invalid since none of our edges have corresponding points.
  EXPECT_FALSE(wireframe.is_valid());

  // ACTION
  for (const auto &point : points_) {
    wireframe.add_point(point);
  }

  // VEIFICATION
  EXPECT_TRUE(wireframe.is_valid());
  EXPECT_EQ(wireframe.points(), points_);
  EXPECT_EQ(wireframe.edges(), edges_);
}

TEST_F(WireframeTest, TestEquality) {
  const Wireframe wireframe_a{points_, edges_};
  Wireframe wireframe_b = wireframe_a;

  EXPECT_EQ(wireframe_a, wireframe_b);

  wireframe_b.add_point(Eigen::Vector3d::Ones());
  EXPECT_NE(wireframe_a, wireframe_b);
  EXPECT_NE(wireframe_b, wireframe_a);

  wireframe_b = wireframe_a;
  wireframe_b.add_edge({0, 1});
  EXPECT_NE(wireframe_a, wireframe_b);
  EXPECT_NE(wireframe_b, wireframe_a);
}

}  // namespace resim::geometry
