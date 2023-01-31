#include <gtest/gtest.h>

#include <utility>

#include "resim_core/transforms/liegroup_test_helpers.hh"
#include "resim_core/transforms/proto/se3.pb.h"
#include "resim_core/transforms/proto/se3_to_proto.hh"
#include "resim_core/transforms/proto/so3.pb.h"
#include "resim_core/transforms/proto/so3_to_proto.hh"
#include "resim_core/transforms/se3.hh"
#include "resim_core/transforms/so3.hh"

namespace resim::transforms {

template <typename T>
class LiegroupToProtoTests : public ::testing::Test {};

using GroupTypePairs =
    ::testing::Types<std::pair<SO3, proto::SO3>, std::pair<SE3, proto::SE3>>;

TYPED_TEST_SUITE(LiegroupToProtoTests, GroupTypePairs);

TYPED_TEST(LiegroupToProtoTests, TestPack) {
  // SETUP
  using Group = typename TypeParam::first_type;
  using GroupMsg = typename TypeParam::second_type;
  GroupMsg msg;
  // ACTION/VERIFICATION
  for (const Group &test_group : make_test_group_elements<Group>()) {
    proto::pack(test_group, &msg);
    EXPECT_EQ(Group::DOF, msg.algebra_size());
    const typename Group::TangentVector test_alg = test_group.log();
    for (unsigned int i = 0; i < Group::DIMS; ++i) {
      EXPECT_DOUBLE_EQ(test_alg[i], msg.algebra(i));
    }
  }
}

TYPED_TEST(LiegroupToProtoTests, TestRoundTrip) {
  // SETUP
  using Group = typename TypeParam::first_type;
  using GroupMsg = typename TypeParam::second_type;
  GroupMsg msg;
  // ACTION/VERIFICATION
  for (const Group &test_group : make_test_group_elements<Group>()) {
    proto::pack(test_group, &msg);
    const Group retrieved_group = proto::unpack(msg);
    EXPECT_TRUE(test_group.is_approx(retrieved_group));
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
    EXPECT_DEATH(
        { proto::pack(test_group, nullptr); },
        "Can't pack into invalid proto!");
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
    EXPECT_DEATH(
        { const Group retrieved_group = unpack(msg); },
        "The expected number of elements in the Matrix");
  }
}
// NOLINTEND(readability-function-cognitive-complexity)

}  //  namespace resim::transforms
