#include "resim_core/utils/match.hh"

#include <gtest/gtest.h>

#include "resim_core/testing/move_copy_tracker.hh"

namespace resim {

using testing::MoveCopyTracker;

constexpr int BACKUP_RETURN = -1;

// This function is a backup overload that's only used when no other overloads
// are present. We include it here to help with the DiscardedOnBadArguments
// test. It's designed to only be selected if the real match() function is
// discarded since its constraints are violated. See the test documentation for
// more information.
template <typename... Ts>
int match(Ts &&...ts) {
  return BACKUP_RETURN;
}

// This function is a helper function for the DiscardedOnBadArguments test
// below. It is an example of a non-functor handler that a user might try to
// pass, but which is not currently supported. See that test documentation for
// for information as to how this is used.
std::string char_handler(const char c) { return std::string{c}; }

TEST(MatchTest, TestMatch) {
  // SETUP
  constexpr char TEST_CHAR = 'm';
  constexpr int TEST_INT = 6;
  constexpr double TEST_DOUBLE = 3.0;
  constexpr bool TEST_BOOL = true;

  enum class Case {
    CHAR = 0,
    INT,
    DEFAULT,
  };

  std::variant<int, char, double, bool> my_variant;
  const auto run_match_rvalues = [&]() {
    return match(
        my_variant,
        [&](const int i) { return Case::INT; },
        [&](const char c) { return Case::CHAR; },
        [&](const auto x) { return Case::DEFAULT; });
  };

  const auto run_match_lvalues = [&]() {
    const auto branch_a = [&](const int i) { return Case::INT; };
    const auto branch_b = [&](const char c) { return Case::CHAR; };
    const auto branch_c = [&](const auto x) { return Case::DEFAULT; };
    return match(my_variant, branch_a, branch_b, branch_c);
  };

  const auto test_match_runner = [&](const auto &run_match) {
    // ACTION / VERIFICATION
    my_variant = TEST_INT;
    EXPECT_EQ(run_match(), Case::INT);

    my_variant = TEST_CHAR;
    EXPECT_EQ(run_match(), Case::CHAR);

    my_variant = TEST_DOUBLE;
    EXPECT_EQ(run_match(), Case::DEFAULT);

    my_variant = TEST_BOOL;
    EXPECT_EQ(run_match(), Case::DEFAULT);
  };

  test_match_runner(run_match_rvalues);
  test_match_runner(run_match_lvalues);
}

TEST(MatchTest, PerfectForwardRvalues) {
  // SETUP
  std::variant<MoveCopyTracker> my_variant;

  // ACTION
  auto &&tracker_rvalue_ref = match(
      std::move(my_variant),
      [&](MoveCopyTracker &&x) -> MoveCopyTracker && { return std::move(x); });

  // VERIFICATION
  EXPECT_EQ(tracker_rvalue_ref.num_copies(), 0);
  EXPECT_EQ(tracker_rvalue_ref.num_moves(), 0);
}

TEST(MatchTest, PerfectForwardLvalues) {
  // SETUP
  const std::variant<MoveCopyTracker> my_variant;

  // ACTION
  const auto &tracker_lvalue_ref = match(
      my_variant,
      [&](const MoveCopyTracker &x) -> const MoveCopyTracker & { return x; });

  // VERIFICATION
  EXPECT_EQ(tracker_lvalue_ref.num_copies(), 0);
  EXPECT_EQ(tracker_lvalue_ref.num_moves(), 0);
}

// This test tests a number of edge cases that violate the constraints
// on the match() function. It does this with the help of the backup
// match() function above. In each case, we pass in invalid arguments
// and verify that the backup function is the only function left in
// the overload set because the match() function under test has been
// correctly discarded.
TEST(MatchTest, DiscardedOnBadArguments) {
  // SETUP
  struct MyObject {};
  const std::variant<char, MyObject> my_variant = 'c';
  constexpr int UNWANTED_RETURN = 1;

  // ACTION / VERIFICATION
  // Not all cases covered
  EXPECT_EQ(BACKUP_RETURN, match(my_variant));
  EXPECT_EQ(BACKUP_RETURN, match(my_variant, [&](const char c) {
              return UNWANTED_RETURN;
            }));

  // Passing in a non_variant
  EXPECT_EQ(BACKUP_RETURN, match(MyObject{}, [&](const char c) {
              return UNWANTED_RETURN;
            }));

  // Passing in a non-object (e.g. a function pointer)
  EXPECT_EQ(BACKUP_RETURN, match(my_variant, &char_handler, [&](const auto c) {
              return UNWANTED_RETURN;
            }));

  //  Lambdas return different types
  EXPECT_EQ(
      BACKUP_RETURN,
      match(
          my_variant,
          [&](const char c) {},
          [&](const auto c) { return UNWANTED_RETURN; }));

  // Conflicting overloads
  EXPECT_EQ(
      BACKUP_RETURN,
      match(
          my_variant,
          [&](const char c) { return UNWANTED_RETURN; },
          [&](const char c) { return UNWANTED_RETURN; },
          [&](const auto c) { return UNWANTED_RETURN; }));
}

}  // namespace resim
