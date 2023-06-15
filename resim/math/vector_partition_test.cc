
#include "resim/math/vector_partition.hh"

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <array>
namespace resim::math {

TEST(VectorPartitionTest, TestOffsetAt) {
  // SETUP
  using Partition = VectorPartition<1, 2, 3>;

  // ACTION
  constexpr auto OFFSETS = std::array{
      OffsetAt<Partition, 0>::value,
      OffsetAt<Partition, 1>::value,
      OffsetAt<Partition, 2>::value,
      OffsetAt<Partition, 3>::value,
  };

  // Fails to compile: with a static assert "Index out of range!"
  // OffsetAt<Partition, 4>::value;

  // VERIFICATION
  EXPECT_EQ(OFFSETS[0U], 0U);
  EXPECT_EQ(OFFSETS[1U], 1U);
  EXPECT_EQ(OFFSETS[2U], 3U);
  EXPECT_EQ(OFFSETS[3U], 6U);
}

// In my judgement, the magic numbers that exist in this test suite promote
// understanding by remaining, since they allow readers to more easily visualize
// the matrices and vectors and the blocks taken from them.
// NOLINTBEGIN(readability-magic-numbers)
TEST(VectorPartitionTest, TestVectorPartitionDim) {
  // SETUP
  using Partition = VectorPartition<1, 2, 3>;

  // ACTION / VERIFICATION
  EXPECT_EQ(VectorPartitionDim<Partition>::value, 6U);
}

TEST(VectorPartitionTest, TestGetBlockVector) {
  // SETUP
  using Partition = VectorPartition<1, 2, 3>;
  using Vector = Eigen::Matrix<double, VectorPartitionDim<Partition>::value, 1>;
  Vector vec;
  vec << 1., 2., 3., 4., 5., 6.;

  constexpr std::size_t TEST_BLOCK_INDEX = 1U;
  {
    // ACTION
    const Eigen::Vector2d observed{get_block<Partition, TEST_BLOCK_INDEX>(vec)};

    // VERIFICATION
    const Eigen::Vector2d expected{2., 3.};
    EXPECT_EQ(observed, expected);
  }
  {
    // ACTION
    get_block<Partition, TEST_BLOCK_INDEX>(vec) *= -1.;

    Vector expected;
    expected << 1., -2., -3., 4., 5., 6.;
    EXPECT_EQ(expected, vec);
  }
}

TEST(VectorPartitionTest, TestGetBlockMatrix) {
  // SETUP
  using RowPartition = VectorPartition<1, 1, 1>;
  using ColPartition = VectorPartition<1, 2>;
  static constexpr int ROW_DIM = VectorPartitionDim<RowPartition>::value;
  static constexpr int COL_DIM = VectorPartitionDim<ColPartition>::value;
  using Matrix = Eigen::Matrix<double, ROW_DIM, COL_DIM>;

  Matrix mat;
  // clang-format off
  mat << 1., 2., 3.,
         4., 5., 6.,
         7., 8., 9.;
  // clang-format on

  constexpr std::size_t ROW_BLOCK_INDEX = 1U;
  constexpr std::size_t COL_BLOCK_INDEX = 1U;
  {
    // ACTION
    const Eigen::RowVector2d observed{
        get_block<RowPartition, ROW_BLOCK_INDEX, ColPartition, COL_BLOCK_INDEX>(
            mat)};

    // VERIFICATION
    const Eigen::RowVector2d expected{5., 6.};
    EXPECT_EQ(observed, expected);
  }
  {
    // ACTION
    get_block<RowPartition, ROW_BLOCK_INDEX, ColPartition, COL_BLOCK_INDEX>(
        mat) *= -1.;

    Matrix expected;
    // clang-format off
    expected << 1., 2., 3.,
                4., -5., -6.,
                7., 8., 9.;
    // clang-format on
    EXPECT_EQ(expected, mat);
  }
}
// NOLINTEND(readability-magic-numbers)

}  // namespace resim::math
