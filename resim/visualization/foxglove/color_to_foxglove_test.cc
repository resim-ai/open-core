
#include "resim/visualization/foxglove/color_to_foxglove.hh"

#include <foxglove/Color.pb.h>
#include <gtest/gtest.h>

#include <random>

namespace resim::visualization::foxglove {

namespace {

// Generate a uniformly random color
template <typename RNG>
Color random_color(RNG &&rng) {
  constexpr double LB = 0.;
  constexpr double UB = 1.;
  std::uniform_real_distribution<double> dist{LB, UB};

  Color result;
  result.r = dist(rng);
  result.g = dist(rng);
  result.b = dist(rng);
  result.a = dist(rng);

  return result;
}

}  // namespace

TEST(ColorToFoxgloveTest, TestPackIntoFoxglove) {
  // SETUP
  constexpr unsigned SEED = 3494U;
  std::mt19937 rng{SEED};

  constexpr int NUM_TESTS = 1000;
  for (int ii = 0; ii < NUM_TESTS; ++ii) {
    const Color test_color{random_color(rng)};

    // ACTION
    ::foxglove::Color color;
    pack_into_foxglove(test_color, &color);

    // VERIFICATION
    EXPECT_EQ(color.r(), test_color.r);
    EXPECT_EQ(color.g(), test_color.g);
    EXPECT_EQ(color.b(), test_color.b);
    EXPECT_EQ(color.a(), test_color.a);
  }
}

}  // namespace resim::visualization::foxglove
