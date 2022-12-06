
#include "resim_core/utils/move_copy_tracker.hh"

#include <gtest/gtest.h>

namespace resim::testing {

TEST(MoveCopyTrackerTest, TestDefaultConstructor) {
  const MoveCopyTracker tracker;
  EXPECT_EQ(tracker.num_moves(), 0U);
  EXPECT_EQ(tracker.num_copies(), 0U);
}

TEST(MoveCopyTrackerTest, TestMove) {
  MoveCopyTracker tracker_a;

  // Assignment
  tracker_a = MoveCopyTracker{};
  EXPECT_EQ(tracker_a.num_moves(), 1U);
  EXPECT_EQ(tracker_a.num_copies(), 0U);

  // Constructor
  MoveCopyTracker tracker_b{std::move(tracker_a)};
  EXPECT_EQ(tracker_b.num_moves(), 2U);
  EXPECT_EQ(tracker_b.num_copies(), 0U);
}

TEST(MoveCopyTrackerTest, TestCopy) {
  const MoveCopyTracker tracker_a;

  // Assignment
  MoveCopyTracker tracker_b;
  tracker_b = tracker_a;
  EXPECT_EQ(tracker_b.num_moves(), 0U);
  EXPECT_EQ(tracker_b.num_copies(), 1U);

  // Constructor
  const MoveCopyTracker tracker_c{tracker_b};
  EXPECT_EQ(tracker_c.num_moves(), 0U);
  EXPECT_EQ(tracker_c.num_copies(), 2U);
}

}  // namespace resim::testing
