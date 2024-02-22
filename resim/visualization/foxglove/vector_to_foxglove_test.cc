#include "resim/visualization/foxglove/vector_to_foxglove.hh"

#include <foxglove/Point3.pb.h>
#include <foxglove/Vector3.pb.h>
#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <random>

#include "resim/testing/random_matrix.hh"

namespace resim::visualization::foxglove {
using Vec3 = Eigen::Vector3d;

template <typename MsgType>
class VectorToFoxgloveTest : public ::testing::Test {
 protected:
  static constexpr unsigned SEED = 89034U;
  std::mt19937 rng_{SEED};
};

using MessageTypes = ::testing::Types<::foxglove::Point3, ::foxglove::Vector3>;
TYPED_TEST_SUITE(VectorToFoxgloveTest, MessageTypes);

TYPED_TEST(VectorToFoxgloveTest, TestPackIntoVector) {
  // SETUP
  constexpr int NUM_TESTS = 1000;
  for (int ii = 0; ii < NUM_TESTS; ++ii) {
    const Vec3 vec{testing::random_vector<Vec3>(TestFixture::rng_)};

    // ACTION
    TypeParam msg;
    pack_into_foxglove(vec, &msg);

    // VERIFICATION
    EXPECT_EQ(msg.x(), vec.x());
    EXPECT_EQ(msg.y(), vec.y());
    EXPECT_EQ(msg.z(), vec.z());
  }
}

}  // namespace resim::visualization::foxglove
