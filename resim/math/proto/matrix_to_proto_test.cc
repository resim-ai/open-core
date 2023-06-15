#include "resim/math/proto/matrix_to_proto.hh"

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <random>

#include "resim/assert/assert.hh"
#include "resim/math/proto/testing/test_matrix.pb.h"
#include "resim/testing/random_matrix.hh"

namespace resim::math {

namespace {
// Some candidate matrix dimensionalities
constexpr unsigned ONE = 1;
constexpr unsigned SML = 5;
constexpr unsigned LRG = 101;
}  // namespace

class MatrixToProtoTest : public ::testing::Test {
 protected:
  std::mt19937 &rng() { return rng_; }

 private:
  static constexpr std::size_t SEED = 97;
  std::mt19937 rng_{SEED};
};

TEST_F(MatrixToProtoTest, PackUnpackVectors) {
  // SETUP
  using TestMatType1 = Eigen::Matrix<double, SML, ONE>;
  using TestMatType2 = Eigen::Matrix<double, ONE, LRG>;
  const TestMatType1 vec_1{testing::random_matrix<TestMatType1>(this->rng())};
  const TestMatType2 vec_2{testing::random_matrix<TestMatType2>(this->rng())};

  // ACTION
  proto::testing::TestMatrix msg;
  proto::pack_matrix(vec_1, msg.mutable_elements());
  TestMatType1 retrieved_1;
  proto::unpack_matrix(msg.elements(), InOut(retrieved_1));

  // VERIFICATION
  EXPECT_TRUE(vec_1.isApprox(retrieved_1));

  // ACTION
  // Test overwiting msg.
  proto::pack_matrix(vec_2, msg.mutable_elements());
  TestMatType2 retrieved_2;
  proto::unpack_matrix(msg.elements(), InOut(retrieved_2));

  // VERIFICATION
  EXPECT_TRUE(vec_2.isApprox(retrieved_2));
}

TEST_F(MatrixToProtoTest, PackUnpackMatrices) {
  // SETUP
  using TestMatType1 = Eigen::Matrix<double, SML, SML>;
  using TestMatType2 = Eigen::Matrix<double, LRG, SML>;
  const TestMatType1 mat_1{testing::random_matrix<TestMatType1>(this->rng())};
  const TestMatType2 mat_2{testing::random_matrix<TestMatType2>(this->rng())};

  // ACTION
  proto::testing::TestMatrix msg;
  proto::pack_matrix(mat_1, msg.mutable_elements());
  TestMatType1 retrieved_1;
  proto::unpack_matrix(msg.elements(), InOut(retrieved_1));

  // VERIFICATION
  EXPECT_TRUE(mat_1.isApprox(retrieved_1));

  // ACTION
  // Test overwiting msg.
  proto::pack_matrix(mat_2, msg.mutable_elements());
  TestMatType2 retrieved_2;
  proto::unpack_matrix(msg.elements(), InOut(retrieved_2));

  // VERIFICATION
  EXPECT_TRUE(mat_2.isApprox(retrieved_2));
}

TEST_F(MatrixToProtoTest, PackUnpackDynamicMatrices) {
  // SETUP
  using TestMatType1 = Eigen::Matrix<double, SML, SML>;
  using TestMatType2 = Eigen::Matrix<double, LRG, SML>;
  const Eigen::MatrixXd mat_1 =
      testing::random_matrix<TestMatType1>(this->rng());
  const Eigen::MatrixXd mat_2 =
      testing::random_matrix<TestMatType2>(this->rng());

  // ACTION
  proto::testing::TestMatrix msg;
  proto::pack_matrix(mat_1, msg.mutable_elements());
  Eigen::MatrixXd retrieved_1(SML, SML);
  proto::unpack_matrix(msg.elements(), InOut(retrieved_1));

  // VERIFICATION
  EXPECT_TRUE(mat_1.isApprox(retrieved_1));

  // ACTION
  // Test overwiting msg.
  proto::pack_matrix(mat_2, msg.mutable_elements());
  Eigen::MatrixXd retrieved_2(LRG, SML);
  proto::unpack_matrix(msg.elements(), InOut(retrieved_2));

  // VERIFICATION
  EXPECT_TRUE(mat_2.isApprox(retrieved_2));
}

using MatrixToProtoDeathTest = MatrixToProtoTest;

TEST_F(MatrixToProtoDeathTest, WrongDimensions) {
  // SETUP
  using TestMatType1 = Eigen::Matrix<double, SML, SML>;
  using TestMatType2 = Eigen::Matrix<double, LRG, SML>;
  const TestMatType1 mat{testing::random_matrix<TestMatType1>(this->rng())};

  // ACTION
  proto::testing::TestMatrix msg;
  proto::pack_matrix(mat, msg.mutable_elements());
  TestMatType2 retrieved;

  // VERIFICATION
  // Unpack into a matix with different dimensions.
  EXPECT_THROW(
      { proto::unpack_matrix(msg.elements(), InOut(retrieved)); },
      AssertException);
}

TEST_F(MatrixToProtoDeathTest, UninitializedDynamic) {
  // SETUP
  using TestMatType1 = Eigen::Matrix<double, SML, SML>;
  const TestMatType1 mat{testing::random_matrix<TestMatType1>(this->rng())};

  // ACTION
  proto::testing::TestMatrix msg;
  proto::pack_matrix(mat, msg.mutable_elements());
  Eigen::MatrixXd retrieved;

  // VERIFICATION
  // Unpack into a dynamic matix with uninitialized size.
  EXPECT_THROW(
      { proto::unpack_matrix(msg.elements(), InOut(retrieved)); },
      AssertException);
}

TEST_F(MatrixToProtoDeathTest, TestPackNull) {
  // SETUP
  using TestMatType1 = Eigen::Matrix<double, SML, SML>;
  const TestMatType1 mat{testing::random_matrix<TestMatType1>(this->rng())};
  // ACTION/VERIFICATION
  EXPECT_THROW({ proto::pack_matrix(mat, nullptr); }, AssertException);
}

}  //  namespace resim::math
