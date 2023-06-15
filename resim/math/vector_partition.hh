#pragma once

#include <Eigen/Dense>
#include <cstdint>

// This class represents a vector which is partitioned into several different
// blocks. An example of this is a state vector that has different components
// for position, velocity, time, etc.
//
//         [ position_x ]
//         [ position_y ]
// state = [ velocity_x ]
//         [ velocity_y ]
//         [   time     ]
//
// Typically, one might want to extract position or velocity from this state
// vector using the Eigen block operators. E.g.
//
// speed = state.segment<2>(2).norm()
//
// This is brittle however since it requires magic numbers and makes it hard to
// add components to the state later. This library promises a better solution by
// defining the block structure exactly once:
//
// enum StateBlockIndices : std::size_t {
//     POSITION = 0U,
//     VELOCITY,
//     TIME,
// };
//
// using StatePartition = VectorPartition<2, 2, 1>;
//
// And then they can access blocks without magic numbers:
//
// speed = get_block<StatePartition, VELOCITY>(state).norm();
//
// In certain cases, especially when dealing with Jacobians, it's useful to be
// able to extract blocks from matrices corresponding to a particular block. For
// example, if we're building a matrix to transform the state and we want it to
// multiply the velocity by 2.0, we can do:
//
// get_block<StatePartition, VELOCITY, StatePartition, VELOCITY>(
//     transform_matrix) = 2.0 * Eigen::Matrix2d::Identity();
//

namespace resim::math {

// This object represents a partition of a vector by storing the sizes of each
// block.
template <std::size_t... BLOCK_SIZES>
struct VectorPartition {};

// Metafunction to get the block size at a given index.
template <typename Partition, std::size_t IDX>
struct BlockSizeAt;

// Recursive case:
// If the index is zero, then return the HEAD block size.
// If the index is not zero, recurse into the tail while reducing the index by
// one.
template <std::size_t HEAD, std::size_t... TAIL, std::size_t IDX>
struct BlockSizeAt<VectorPartition<HEAD, TAIL...>, IDX> {
  static_assert(IDX <= sizeof...(TAIL), "Index out of range!");
  static constexpr std::size_t value =
      (IDX == 0U)
          ? HEAD
          : BlockSizeAt<VectorPartition<TAIL...>, (IDX > 0U ? IDX - 1U : 0U)>::
                value;
};

// Base case
template <>
struct BlockSizeAt<VectorPartition<>, 0U> {
  static constexpr std::size_t value = 0U;
};

// Metafunction to get the block offset at a given index
template <typename Partition, std::size_t IDX>
struct OffsetAt;

// Recursive case:
// If the index is zero, then return 0U since we have no more to add to the
// current offset.
// If the index is not zero, add the current block to the offset sum.
// Example:
//
// offset_at([1, 1, 3, 2], 3) = 1 + offset_at([1, 3, 2], 2)
//                            = 1 + 1 + offset_at([3, 2], 1)
//                            = 1 + 1 + 3 + offset_at([2], 0)
//                            = 1 + 1 + 3 + 0
//                            = 5
//
// Note that we allow this to be queried at index equal to the size of the
// vector partition, in which case, we march past the final block and return the
// total size of the vector being partitioned. In other terms, we return the
// first offset which is "past the end" of the vector being partitioned.
template <std::size_t HEAD, std::size_t... TAIL, std::size_t IDX>
struct OffsetAt<VectorPartition<HEAD, TAIL...>, IDX> {
  static_assert(IDX <= sizeof...(TAIL) + 1U, "Index out of range!");
  static constexpr std::size_t value =
      (IDX == 0U)
          ? 0U
          : HEAD +
                OffsetAt<VectorPartition<TAIL...>, (IDX > 0U ? IDX - 1U : 0U)>::
                    value;
};

template <std::size_t IDX>
struct OffsetAt<VectorPartition<>, IDX> {
  static constexpr std::size_t value = 0U;
};

// Metafunction to return the dimension of the vector being partitioned. We use
// the "past the end" behavior of the OffsetAt metafunction described above to
// compute this equally.
template <typename Partition>
struct VectorPartitionDim;

template <std::size_t... BLOCK_SIZES>
struct VectorPartitionDim<VectorPartition<BLOCK_SIZES...>>
    : OffsetAt<VectorPartition<BLOCK_SIZES...>, sizeof...(BLOCK_SIZES)> {};

// This function allows a user to get a block out of a given vector as described
// above.
// @tparam[in] Partition - The partition describing the blocks.
// @tparam[in] BlockIndex - The index of the block in Partition that we want.
// @param[in] vector - The vector to get the block out of.
template <typename Partition, std::size_t BlockIndex, typename VectorType>
Eigen::VectorBlock<VectorType, BlockSizeAt<Partition, BlockIndex>::value>
get_block(VectorType &vector) {
  static constexpr std::size_t OFFSET = OffsetAt<Partition, BlockIndex>::value;
  static constexpr std::size_t SIZE = BlockSizeAt<Partition, BlockIndex>::value;
  return vector.template segment<SIZE>(OFFSET);
}

// This function allows a user to get a block out of a given matrix as described
// above.
// @tparam[in] RowPartition - The partition describing the row blocks.
// @tparam[in] RowBlockIndex - The index of the block in RowPartition that we
//                             want.
// @tparam[in] ColPartition - The partition describing the row blocks.
// @tparam[in] ColBlockIndex - The index of the block in ColPartition that we
//                             want.
// @param[in] matrix- The matrix to get the block out of.
template <
    typename RowPartition,
    std::size_t RowBlockIndex,
    typename ColPartition,
    std::size_t ColBlockIndex,
    typename MatrixType>
Eigen::Block<
    MatrixType,
    BlockSizeAt<RowPartition, RowBlockIndex>::value,
    BlockSizeAt<ColPartition, ColBlockIndex>::value>
get_block(MatrixType &matrix) {
  static constexpr std::size_t ROW_OFFSET =
      OffsetAt<RowPartition, RowBlockIndex>::value;
  static constexpr std::size_t COL_OFFSET =
      OffsetAt<ColPartition, ColBlockIndex>::value;
  static constexpr std::size_t ROWS =
      BlockSizeAt<RowPartition, RowBlockIndex>::value;
  static constexpr std::size_t COLS =
      BlockSizeAt<ColPartition, ColBlockIndex>::value;
  return matrix.template block<ROWS, COLS>(ROW_OFFSET, COL_OFFSET);
}

}  // namespace resim::math
