// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/transforms/framed_vector.hh"

#include <gtest/gtest.h>

#include <random>
#include <type_traits>

#include "resim/testing/random_matrix.hh"

namespace resim::transforms {

namespace {
constexpr unsigned THREE_D = 3;
}  // namespace

template <typename T>
class FramedVectorTest : public ::testing::Test {
 protected:
  Eigen::Matrix<double, T::value, 1> generate_test_vector() {
    return testing::random_vector<Eigen::Matrix<double, T::value, 1>>(rng_);
  }

 private:
  std::mt19937 rng_;
};

using DimensionalityTypes =
    ::testing::Types<std::integral_constant<unsigned int, THREE_D>>;

TYPED_TEST_SUITE(FramedVectorTest, DimensionalityTypes);

TYPED_TEST(FramedVectorTest, Construction) {
  // SETUP
  constexpr unsigned D = TypeParam::value;
  const Frame<D> test_frame = Frame<D>::new_frame();
  const Eigen::Matrix<double, D, 1> test_vector = this->generate_test_vector();

  // ACTION
  // Initialize with using only a vector.
  const FramedVector<D> test_framed_vector(this->generate_test_vector());
  // Initialize with a frame.
  const FramedVector<D> test_framed_vector_2(test_vector, test_frame);

  // VERIFICATION
  EXPECT_NE(test_frame, test_framed_vector.frame());
  EXPECT_EQ(test_frame, test_framed_vector_2.frame());
  // Direct comparison of framed and unframed vectors should work.
  EXPECT_EQ(test_vector, test_framed_vector_2);

  // Uncomment below to test compile time errors at construction.
  // const Eigen::Matrix<double, D-1, 1> bad_vector{3.142, 2.718};
  // const FramedVector<D> bad_framed_vector(bad_vector);

  // const Frame<D-1> bad_frame = Frame<D-1>::new_frame();
  // const FramedVector<D> bad_framed_vector_2(test_vector, bad_frame);
}

TYPED_TEST(FramedVectorTest, VectorAddition) {
  // SETUP
  constexpr unsigned D = TypeParam::value;
  const Frame<D> test_frame = Frame<D>::new_frame();
  const Eigen::Matrix<double, D, 1> test_vector_1 =
      this->generate_test_vector();
  const Eigen::Matrix<double, D, 1> test_vector_2 =
      this->generate_test_vector();

  // ACTION
  const FramedVector<D> test_framed_vector_1(test_vector_1, test_frame);
  const FramedVector<D> test_framed_vector_2(test_vector_2, test_frame);
  const FramedVector<D> resultant_framed_vector =
      test_framed_vector_1 + test_framed_vector_2;
  const Eigen::Matrix<double, D, 1> resultant_unframed_1 =
      test_framed_vector_1 + test_vector_2;
  const Eigen::Matrix<double, D, 1> resultant_unframed_2 =
      test_vector_1 + test_framed_vector_2;

  // VERIFICATION
  EXPECT_EQ(test_frame, resultant_framed_vector.frame());
  // Direct comparison of framed and unframed vectors should work.
  EXPECT_EQ(resultant_framed_vector, resultant_unframed_1);
  EXPECT_EQ(resultant_framed_vector, resultant_unframed_2);
  const Eigen::Matrix<double, D, 1> sum = test_vector_1 + test_vector_2;
  EXPECT_EQ(resultant_framed_vector, sum);
}

TYPED_TEST(FramedVectorTest, ComparisonOperations) {
  // SETUP
  constexpr unsigned D = TypeParam::value;
  const Frame<D> test_frame = Frame<D>::new_frame();
  const Eigen::Matrix<double, D, 1> test_vector_1 =
      this->generate_test_vector();
  const Eigen::Matrix<double, D, 1> test_vector_2 =
      this->generate_test_vector();

  // ACTION
  const FramedVector<D> test_framed_vector_1(test_vector_1, test_frame);
  const FramedVector<D> test_framed_vector_2(test_vector_2, test_frame);
  const FramedVector<D> test_framed_vector_3(test_vector_1);
  const FramedVector<D>& copy_framed_vector_1(test_framed_vector_1);

  // VERIFY EXACT EQUALITIES
  EXPECT_EQ(test_framed_vector_1, copy_framed_vector_1);
  EXPECT_EQ(test_framed_vector_1, test_vector_1);
  EXPECT_NE(test_framed_vector_1, test_framed_vector_2);
  EXPECT_NE(test_framed_vector_1, test_framed_vector_3);
  EXPECT_EQ(test_framed_vector_1.vector(), test_framed_vector_3);
  // Cover the != operator.
  EXPECT_TRUE(test_framed_vector_1 != test_framed_vector_3);

  // VERIFY APPROX EQUALITIES
  EXPECT_TRUE(test_framed_vector_1.isApprox(copy_framed_vector_1));
  EXPECT_TRUE(test_framed_vector_1.isApprox(test_vector_1));
  EXPECT_TRUE(test_vector_1.isApprox(test_framed_vector_1));
  EXPECT_FALSE(test_framed_vector_1.isApprox(test_framed_vector_2));
  EXPECT_FALSE(test_framed_vector_1.isApprox(test_framed_vector_3));
  EXPECT_TRUE(test_framed_vector_1.vector().isApprox(test_framed_vector_3));
}

TYPED_TEST(FramedVectorTest, SetFrame) {
  // SETUP
  constexpr unsigned D = TypeParam::value;
  const Frame<D> test_frame = Frame<D>::new_frame();
  const Eigen::Matrix<double, D, 1> test_vector = this->generate_test_vector();

  // ACTION/VERIFICATION
  FramedVector<D> test_framed_vector(test_vector);
  EXPECT_NE(test_frame, test_framed_vector.frame());
  test_framed_vector.set_frame(test_frame);
  EXPECT_EQ(test_frame, test_framed_vector.frame());
}

TYPED_TEST(FramedVectorTest, ScalarMulitply) {
  // SETUP
  constexpr unsigned D = TypeParam::value;
  const Eigen::Matrix<double, D, 1> test_vector = this->generate_test_vector();

  // ACTION
  const FramedVector<D> test_framed_vector(test_vector);
  const FramedVector<D> result_framed_vector_right = test_framed_vector * 0.5;
  const FramedVector<D> result_framed_vector_left = 0.5 * test_framed_vector;

  // VERIFICATION
  EXPECT_EQ(result_framed_vector_right.frame(), test_framed_vector.frame());
  EXPECT_EQ(result_framed_vector_left.frame(), test_framed_vector.frame());
  EXPECT_EQ(result_framed_vector_left, result_framed_vector_right);
}

template <typename T>
using FramedVectorAssertionTest = FramedVectorTest<T>;
TYPED_TEST_SUITE(FramedVectorAssertionTest, DimensionalityTypes);

TYPED_TEST(FramedVectorAssertionTest, VectorAddition) {
  // SETUP
  constexpr unsigned D = TypeParam::value;
  const Frame<D> test_frame = Frame<D>::new_frame();
  const Eigen::Matrix<double, D, 1> test_vector_1 =
      this->generate_test_vector();
  const Eigen::Matrix<double, D, 1> test_vector_2 =
      this->generate_test_vector();

  // ACTION
  const FramedVector<D> test_framed_vector_1(test_vector_1);
  const FramedVector<D> test_framed_vector_2(test_vector_2, test_frame);
  ASSERT_NE(test_framed_vector_1.frame(), test_framed_vector_2.frame());

  // VERIFICATION
  EXPECT_THROW(
      {
        const FramedVector<D> resultant_framed_vector =
            test_framed_vector_1 + test_framed_vector_2;
        (void)resultant_framed_vector;  // Avoid unused variable errors.
      },
      AssertException);
}

}  // namespace resim::transforms
