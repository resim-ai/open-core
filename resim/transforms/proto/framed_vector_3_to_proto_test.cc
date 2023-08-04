// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/transforms/proto/framed_vector_3_to_proto.hh"

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include "resim/assert/assert.hh"
#include "resim/testing/random_matrix.hh"

namespace resim::transforms {

namespace {
constexpr unsigned THREE_D = 3;
constexpr unsigned int SEED = 349;
}  // namespace

// Only Frame3 is supported, so we will only test that.
class FramedVector3ToProtoTests : public ::testing::Test {
 protected:
  Eigen::Matrix<double, THREE_D, 1> generate_test_vector() {
    return testing::random_vector<Eigen::Matrix<double, THREE_D, 1>>(rng_);
  }

 private:
  std::mt19937 rng_{SEED};
};

TEST_F(FramedVector3ToProtoTests, TestPack) {
  // SETUP
  const auto test_frame = Frame<THREE_D>::new_frame();
  const auto test_vector = this->generate_test_vector();
  const FramedVector<THREE_D> test_framed_vector(test_vector, test_frame);

  // ACTION
  proto::FramedVector_3 msg;
  proto::pack(test_framed_vector, &msg);

  // VERIFICATION
  EXPECT_EQ(msg.frame().id().data(), test_frame.id().to_string());
  EXPECT_EQ(msg.algebra_size(), 3);
  for (unsigned int i = 0; i < msg.algebra_size(); ++i) {
    EXPECT_DOUBLE_EQ(msg.algebra(i), test_vector[i]);
  }
}

TEST_F(FramedVector3ToProtoTests, TestRoundTrip) {
  // SETUP
  const auto test_frame = Frame<THREE_D>::new_frame();
  const auto test_vector = this->generate_test_vector();
  const FramedVector<THREE_D> test_framed_vector(test_vector, test_frame);

  // ACTION
  proto::FramedVector_3 msg;
  proto::pack(test_framed_vector, &msg);
  FramedVector<THREE_D> retrieved_vector = proto::unpack(msg);

  // VERIFICATION
  EXPECT_EQ(
      retrieved_vector.frame().id().to_string(),
      test_framed_vector.frame().id().to_string());
  EXPECT_EQ(retrieved_vector, test_framed_vector);
}

using FramedVector3ToProtoTestsDeathTests = FramedVector3ToProtoTests;

TEST_F(FramedVector3ToProtoTestsDeathTests, TestPackNull) {
  const auto test_frame = Frame<THREE_D>::new_frame();
  const auto test_vector = this->generate_test_vector();
  const FramedVector<THREE_D> test_framed_vector(test_vector, test_frame);

  EXPECT_THROW({ proto::pack(test_framed_vector, nullptr); }, AssertException);
}

}  //  namespace resim::transforms
