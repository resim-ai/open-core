#include "resim/ros2/point_from_ros2.hh"

#include <gtest/gtest.h>

#include <random>

#include "resim/converter/fuzz_helpers.hh"
#include "resim/transforms/frame.hh"
#include "resim/transforms/proto/framed_vector_3.pb.h"
#include "resim/transforms/proto/fuzz_helpers.hh"
#include "resim/utils/inout.hh"

namespace resim::ros2 {

TEST(PointFromRos2Test, TestRoundTrip) {
  // SETUP
  constexpr std::size_t SEED = 85830U;
  std::mt19937 rng{SEED};
  transforms::proto::FramedVector_3 test_vector{
      converter::random_element<transforms::proto::FramedVector_3>(InOut{rng})};
  // We don't use frame IDs when converting to/from ROS2.
  constexpr int DIMS = 3;
  test_vector.mutable_frame()->mutable_id()->set_data(
      transforms::Frame<DIMS>::null_frame().id().to_string());

  // ACTION
  const transforms::proto::FramedVector_3 round_trip{
      convert_from_ros2(convert_to_ros2(test_vector))};

  // VERIFICATION
  EXPECT_TRUE(converter::verify_equality(test_vector, round_trip));
}

}  // namespace resim::ros2
