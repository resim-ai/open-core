// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <gtest/gtest.h>

#include <cstdlib>
#include <utility>

#include "resim/assert/assert.hh"
#include "resim/transforms/frame.hh"
#include "resim/transforms/liegroup_test_helpers.hh"
#include "resim/transforms/proto/se3.pb.h"
#include "resim/transforms/proto/se3_to_proto.hh"
#include "resim/transforms/proto/so3.pb.h"
#include "resim/transforms/proto/so3_to_proto.hh"
#include "resim/transforms/se3.hh"
#include "resim/transforms/so3.hh"

namespace resim::transforms {

namespace {
// Create some test frames.
constexpr unsigned int DIMS = 3;
const Frame<DIMS> A = Frame<DIMS>::new_frame();
const Frame<DIMS> B = Frame<DIMS>::new_frame();
const Frame<DIMS> C = Frame<DIMS>::new_frame();
const Frame<DIMS> N = Frame<DIMS>();  // Null Frame
}  // namespace

template <typename T>
class LiegroupToProtoTests : public ::testing::Test {};

using GroupTypePairs =
    ::testing::Types<std::pair<SO3, proto::SO3>, std::pair<SE3, proto::SE3>>;

TYPED_TEST_SUITE(LiegroupToProtoTests, GroupTypePairs);

// NOLINTBEGIN(readability-function-cognitive-complexity)
TYPED_TEST(LiegroupToProtoTests, TestPackWithNullFrames) {
  // SETUP
  using Group = typename TypeParam::first_type;
  using GroupMsg = typename TypeParam::second_type;
  GroupMsg msg;
  // ACTION/VERIFICATION
  for (const Group &test_group : make_test_group_elements<Group>()) {
    // Confirm frames are null
    ASSERT_EQ(test_group.into(), N);
    ASSERT_EQ(test_group.from(), N);
    proto::pack(test_group, &msg);
    EXPECT_EQ(Group::DOF, msg.algebra_size());
    const typename Group::TangentVector test_alg = test_group.log();
    for (unsigned int i = 0; i < Group::DIMS; ++i) {
      EXPECT_DOUBLE_EQ(test_alg[i], msg.algebra(i));
    }
    EXPECT_EQ(test_group.into().id().to_string(), msg.into().id().data());
    EXPECT_EQ(test_group.from().id().to_string(), msg.from().id().data());
  }
}
// NOLINTEND(readability-function-cognitive-complexity)

TYPED_TEST(LiegroupToProtoTests, TestPackWithFrames) {
  // SETUP
  using Group = typename TypeParam::first_type;
  using GroupMsg = typename TypeParam::second_type;
  GroupMsg msg;
  // ACTION/VERIFICATION
  for (Group &test_group : make_test_group_elements<Group>()) {
    test_group.set_frames(A, B);
    proto::pack(test_group, &msg);
    EXPECT_EQ(Group::DOF, msg.algebra_size());
    const typename Group::TangentVector test_alg = test_group.log();
    for (unsigned int i = 0; i < Group::DIMS; ++i) {
      EXPECT_DOUBLE_EQ(test_alg[i], msg.algebra(i));
    }
    EXPECT_EQ(test_group.into().id().to_string(), msg.into().id().data());
    EXPECT_EQ(test_group.from().id().to_string(), msg.from().id().data());
  }
}

TYPED_TEST(LiegroupToProtoTests, TestRoundTripWithNullFrames) {
  // SETUP
  using Group = typename TypeParam::first_type;
  using GroupMsg = typename TypeParam::second_type;
  GroupMsg msg;
  // ACTION/VERIFICATION
  for (const Group &test_group : make_test_group_elements<Group>()) {
    ASSERT_EQ(test_group.into(), N);
    ASSERT_EQ(test_group.from(), N);
    proto::pack(test_group, &msg);
    const Group retrieved_group = proto::unpack(msg);
    EXPECT_TRUE(test_group.is_approx(retrieved_group));
    // is_approx also checks frames, but let's double check and also
    // cover some negative cases.
    EXPECT_TRUE(test_group.verify_frames(N, N));
    // Verify negative case on frames.
    EXPECT_FALSE(test_group.verify_frames(A, C));
    EXPECT_FALSE(test_group.verify_frames(C, B));
  }
}

TYPED_TEST(LiegroupToProtoTests, TestRoundTripWithFrames) {
  // SETUP
  using Group = typename TypeParam::first_type;
  using GroupMsg = typename TypeParam::second_type;
  GroupMsg msg;
  // ACTION/VERIFICATION
  for (Group &test_group : make_test_group_elements<Group>()) {
    test_group.set_frames(A, B);
    proto::pack(test_group, &msg);
    const Group retrieved_group = proto::unpack(msg);
    EXPECT_TRUE(test_group.is_approx(retrieved_group));
    // is_approx also checks frames, but let's double check and also
    // cover some negative cases.
    EXPECT_TRUE(test_group.verify_frames(A, B));
    // Verify negative case on frames.
    EXPECT_FALSE(test_group.verify_frames(A, C));
    EXPECT_FALSE(test_group.verify_frames(C, B));
  }
}

template <typename T>
using LiegroupToProtoDeathTests = LiegroupToProtoTests<T>;
TYPED_TEST_SUITE(LiegroupToProtoDeathTests, GroupTypePairs);

// NOLINTBEGIN(readability-function-cognitive-complexity)
TYPED_TEST(LiegroupToProtoDeathTests, TestPackNull) {
  // SETUP
  using Group = typename TypeParam::first_type;
  // ACTION/VERIFICATION
  for (const Group &test_group : make_test_group_elements<Group>()) {
    EXPECT_THROW({ proto::pack(test_group, nullptr); }, AssertException);
  }
}

TYPED_TEST(LiegroupToProtoDeathTests, TestLongMessage) {
  // SETUP
  using Group = typename TypeParam::first_type;
  using GroupMsg = typename TypeParam::second_type;
  GroupMsg msg;
  // ACTION/VERIFICATION
  for (const Group &test_group : make_test_group_elements<Group>()) {
    proto::pack(test_group, &msg);
    // Pack another element.
    constexpr double ROGUE_ELEMENT = 1.4142;
    msg.add_algebra(ROGUE_ELEMENT);
    EXPECT_THROW(
        { const Group retrieved_group = unpack(msg); },
        AssertException);
  }
}
// NOLINTEND(readability-function-cognitive-complexity)

}  //  namespace resim::transforms
