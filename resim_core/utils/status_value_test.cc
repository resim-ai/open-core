#include "resim_core/utils/status_value.hh"

#include <gtest/gtest.h>

#include <string>
#include <string_view>

#include "resim_core/testing/move_copy_tracker.hh"

namespace resim {

namespace {

// This helper class allows us to tracks moves and copies while also carrying
// around some non-default data we can use for comparison. This lets us make
// sure we aren't just getting back a default-constructed version of this when
// we query it from a StatusValue containing one.
class MoveCopyTrackerWithInt : public testing::MoveCopyTracker {
 public:
  explicit MoveCopyTrackerWithInt(const int x) : x_{x} {}

  int x() const { return x_; }

 private:
  int x_ = 0;
};

constexpr int NON_DEFAULT_INTEGER = 3;

// This helper determines whether a given StatusValue containing a
// MoveCopyTrackerWithInt or a reference to one matches up with the
// NON_DEFAULT_INTEGER defined above.
template <typename T>
void expect_matches_tracker_value(const T &tracker_status_value) {
  EXPECT_EQ(tracker_status_value.value().x(), NON_DEFAULT_INTEGER);
  EXPECT_TRUE(tracker_status_value.ok());
  EXPECT_EQ(&tracker_status_value.status(), &OKAY_STATUS);
}

// A helper function which calls RETURN_IF_NOT_OK() so we can test it.
template <typename T>
Status return_if_not_ok_function(const StatusValue<T> &sv) {
  RETURN_IF_NOT_OK(sv);
  return OKAY_STATUS;
}

// A helper function which calls RETURN_OR_ASSIGN() so we can test it.
template <typename StatusValueType>
Status return_or_assign_function(StatusValueType &&sv) {
  const typename std::decay_t<StatusValueType>::ValueType val =
      RETURN_OR_ASSIGN(std::forward<StatusValueType>(sv));
  EXPECT_TRUE(sv.ok());
  EXPECT_EQ(val, sv.value());
  return OKAY_STATUS;
}

}  // namespace

// Test that we can construct a StatusValue<T> based on a value.
TEST(StatusValueTest, TestConstructValue) {
  {
    // StatusValue<T> from T lvalue
    MoveCopyTrackerWithInt tracker{NON_DEFAULT_INTEGER};
    const StatusValue<MoveCopyTrackerWithInt> tracker_status_value{tracker};
    EXPECT_EQ(tracker_status_value.value().num_moves(), 0);
    EXPECT_EQ(tracker_status_value.value().num_copies(), 1);
    expect_matches_tracker_value(tracker_status_value);
  }
  {
    // StatusValue<T> from const T lvalue
    const MoveCopyTrackerWithInt tracker{NON_DEFAULT_INTEGER};
    const StatusValue<MoveCopyTrackerWithInt> tracker_status_value{tracker};
    EXPECT_EQ(tracker_status_value.value().num_moves(), 0);
    EXPECT_EQ(tracker_status_value.value().num_copies(), 1);
    expect_matches_tracker_value(tracker_status_value);
  }
  {
    // StatusValue<T> from T rvalue
    MoveCopyTrackerWithInt tracker{NON_DEFAULT_INTEGER};
    const StatusValue<MoveCopyTrackerWithInt> tracker_status_value{
        std::move(tracker)};
    EXPECT_EQ(tracker_status_value.value().num_moves(), 1);
    EXPECT_EQ(tracker_status_value.value().num_copies(), 0);
    expect_matches_tracker_value(tracker_status_value);
  }
  {
    // StatusValue<T> from const T rvalue
    const MoveCopyTrackerWithInt tracker{NON_DEFAULT_INTEGER};
    const StatusValue<MoveCopyTrackerWithInt> tracker_status_value{
        static_cast<const MoveCopyTrackerWithInt &&>(tracker)};
    EXPECT_EQ(tracker_status_value.value().num_moves(), 0);
    EXPECT_EQ(tracker_status_value.value().num_copies(), 1);  // Can't be moved
    expect_matches_tracker_value(tracker_status_value);
  }
}

// Test that we can construct a StatusValue<T &> based on a value.
TEST(StatusValueTest, TestConstructLvalueRef) {
  // StatusValue<T &> from T lvalue
  MoveCopyTrackerWithInt tracker{NON_DEFAULT_INTEGER};
  const StatusValue<MoveCopyTrackerWithInt &> tracker_status_value{tracker};
  EXPECT_EQ(tracker_status_value.value().num_moves(), 0);
  EXPECT_EQ(tracker_status_value.value().num_copies(), 0);
  expect_matches_tracker_value(tracker_status_value);
  EXPECT_EQ(&tracker_status_value.value(), &tracker);

  // Can't bind const lvalues or any rvalues
}

// Test that we can construct a StatusValue<const T &> based on a value.
TEST(StatusValueTest, TestConstructConstLvalueRef) {
  {
    // StatusValue<const T &> from T lvalue
    MoveCopyTrackerWithInt tracker{NON_DEFAULT_INTEGER};
    const StatusValue<const MoveCopyTrackerWithInt &> tracker_status_value{
        tracker};
    EXPECT_EQ(tracker_status_value.value().num_moves(), 0);
    EXPECT_EQ(tracker_status_value.value().num_copies(), 0);
    expect_matches_tracker_value(tracker_status_value);
    EXPECT_EQ(&tracker_status_value.value(), &tracker);
  }
  {
    // StatusValue<const T &> from const T lvalue
    const MoveCopyTrackerWithInt tracker{NON_DEFAULT_INTEGER};
    const StatusValue<const MoveCopyTrackerWithInt &> tracker_status_value{
        tracker};
    EXPECT_EQ(tracker_status_value.value().num_moves(), 0);
    EXPECT_EQ(tracker_status_value.value().num_copies(), 0);
    expect_matches_tracker_value(tracker_status_value);
    EXPECT_EQ(&tracker_status_value.value(), &tracker);
  }
  {
    // StatusValue<const T &> from T rvalue
    MoveCopyTrackerWithInt tracker{NON_DEFAULT_INTEGER};
    const StatusValue<const MoveCopyTrackerWithInt &> tracker_status_value{
        std::move(tracker)};
    EXPECT_EQ(
        tracker_status_value.value().num_moves(),
        0);  // Lifetime extension
    EXPECT_EQ(tracker_status_value.value().num_copies(), 0);
    expect_matches_tracker_value(tracker_status_value);
  }
  {
    // StatusValue<const T &> from const T rvalue
    MoveCopyTrackerWithInt tracker{NON_DEFAULT_INTEGER};
    const StatusValue<const MoveCopyTrackerWithInt &> tracker_status_value{
        static_cast<const MoveCopyTrackerWithInt &&>(tracker)};
    EXPECT_EQ(
        tracker_status_value.value().num_moves(),
        0);  // Lifetime extension
    EXPECT_EQ(tracker_status_value.value().num_copies(), 0);
    expect_matches_tracker_value(tracker_status_value);
  }
}

// Test that we can construct a StatusValue<T &&> based on a value.
TEST(StatusValueTest, TestConstructRvalueRef) {
  // StatusValue<T &&> from T rvalue
  MoveCopyTrackerWithInt tracker{NON_DEFAULT_INTEGER};
  const StatusValue<MoveCopyTrackerWithInt &&> tracker_status_value{
      std::move(tracker)};
  EXPECT_EQ(tracker_status_value.value().num_moves(), 0);
  EXPECT_EQ(tracker_status_value.value().num_copies(), 0);
  expect_matches_tracker_value(tracker_status_value);
  // Can't bind const rvalues or any lvalues
}

// Test that we can construct a StatusValue<const T &&> based on a value.
TEST(StatusValueTest, TestConstructConstRvalueRef) {
  {
    // StatusValue<const T &&> from T rvalue
    MoveCopyTrackerWithInt tracker{NON_DEFAULT_INTEGER};
    const StatusValue<const MoveCopyTrackerWithInt &&> tracker_status_value{
        std::move(tracker)};
    EXPECT_EQ(tracker_status_value.value().num_moves(), 0);
    EXPECT_EQ(tracker_status_value.value().num_copies(), 0);
    expect_matches_tracker_value(tracker_status_value);
  }
  {
    // StatusValue<const T &&> from const T rvalue
    MoveCopyTrackerWithInt tracker{NON_DEFAULT_INTEGER};
    const StatusValue<const MoveCopyTrackerWithInt &&> tracker_status_value{
        static_cast<const MoveCopyTrackerWithInt &&>(tracker)};
    EXPECT_EQ(tracker_status_value.value().num_moves(), 0);
    EXPECT_EQ(tracker_status_value.value().num_copies(), 0);
    expect_matches_tracker_value(tracker_status_value);
  }
  // Can't bind lvalues
}

// Test that we can construct a StatusValue<T> based on a status
TEST(StatusValueTest, TestConstructWithStatus) {
  const auto status = MAKE_STATUS("Test status!");
  {
    const StatusValue<MoveCopyTrackerWithInt> tracker{status};
    EXPECT_FALSE(tracker.ok());
    EXPECT_EQ(tracker.status().what(), status.what());
  }
  {
    const StatusValue<MoveCopyTrackerWithInt &> tracker{status};
    EXPECT_FALSE(tracker.ok());
    EXPECT_EQ(tracker.status().what(), status.what());
  }

  {
    const StatusValue<const MoveCopyTrackerWithInt &> tracker{status};
    EXPECT_FALSE(tracker.ok());
    EXPECT_EQ(tracker.status().what(), status.what());
  }

  {
    const StatusValue<MoveCopyTrackerWithInt &&> tracker{status};
    EXPECT_FALSE(tracker.ok());
    EXPECT_EQ(tracker.status().what(), status.what());
  }
  {
    const StatusValue<const MoveCopyTrackerWithInt &&> tracker{status};
    EXPECT_FALSE(tracker.ok());
    EXPECT_EQ(tracker.status().what(), status.what());
  }
}

// Test that we fail when trying to create a StatusValue from an okay
// status. This is not allowed as it would yield an unpopulated StatusValue
TEST(StatusValueDeathTest, TestCantConstructFromOkayStatus) {
  EXPECT_DEATH(
      { StatusValue<MoveCopyTrackerWithInt>{OKAY_STATUS}; },
      "Can't make okay StatusValue without value!");

  EXPECT_DEATH(
      { StatusValue<MoveCopyTrackerWithInt &>{OKAY_STATUS}; },
      "Can't make okay StatusValue without value!");

  EXPECT_DEATH(
      { StatusValue<const MoveCopyTrackerWithInt &>{OKAY_STATUS}; },
      "Can't make okay StatusValue without value!");

  EXPECT_DEATH(
      { StatusValue<MoveCopyTrackerWithInt &&>{OKAY_STATUS}; },
      "Can't make okay StatusValue without value!");

  EXPECT_DEATH(
      { StatusValue<const MoveCopyTrackerWithInt &&>{OKAY_STATUS}; },
      "Can't make okay StatusValue without value!");
}

// Test that we can create StatusValues with copying.
TEST(StatusValueTest, TestCopyConstructor) {
  {
    // StatusValue<T>
    const MoveCopyTrackerWithInt tracker{NON_DEFAULT_INTEGER};
    const StatusValue<MoveCopyTrackerWithInt> tracker_status_value{tracker};

    // This copy initialization is sortof the point
    // NOLINTNEXTLINE(performance-unnecessary-copy-initialization)
    const StatusValue<MoveCopyTrackerWithInt> tracker_status_value_copy{
        tracker_status_value};
    EXPECT_EQ(tracker_status_value_copy.value().num_moves(), 0);
    EXPECT_EQ(tracker_status_value_copy.value().num_copies(), 2);
    expect_matches_tracker_value(tracker_status_value_copy);
  }
  {
    // StatusValue<T &>
    MoveCopyTrackerWithInt tracker{NON_DEFAULT_INTEGER};
    const StatusValue<MoveCopyTrackerWithInt &> tracker_status_value{tracker};
    const StatusValue<MoveCopyTrackerWithInt &> tracker_status_value_copy{
        tracker_status_value};
    EXPECT_EQ(tracker_status_value_copy.value().num_moves(), 0);
    EXPECT_EQ(tracker_status_value_copy.value().num_copies(), 0);
    expect_matches_tracker_value(tracker_status_value_copy);
    EXPECT_EQ(&tracker_status_value_copy.value(), &tracker);
  }
  {
    // StatusValue<const T &>
    const MoveCopyTrackerWithInt tracker{NON_DEFAULT_INTEGER};
    const StatusValue<const MoveCopyTrackerWithInt &> tracker_status_value{
        tracker};
    const StatusValue<const MoveCopyTrackerWithInt &> tracker_status_value_copy{
        tracker_status_value};
    EXPECT_EQ(tracker_status_value_copy.value().num_moves(), 0);
    EXPECT_EQ(tracker_status_value_copy.value().num_copies(), 0);
    expect_matches_tracker_value(tracker_status_value_copy);
    EXPECT_EQ(&tracker_status_value_copy.value(), &tracker);
  }
  // StatusValue<T &&> and StatusValue<const T &&> have their copy
  // constructors deleted since rvalue references cannot be copied.
}

// Test that we can create all manner of StatusValues with moving
TEST(StatusValueTest, TestMoveConstructor) {
  {
    // StatusValue<T>
    const MoveCopyTrackerWithInt tracker{NON_DEFAULT_INTEGER};
    StatusValue<MoveCopyTrackerWithInt> tracker_status_value{tracker};
    const StatusValue<MoveCopyTrackerWithInt> tracker_status_value_move{
        std::move(tracker_status_value)};
    EXPECT_EQ(tracker_status_value_move.value().num_moves(), 1);
    EXPECT_EQ(tracker_status_value_move.value().num_copies(), 1);
    expect_matches_tracker_value(tracker_status_value_move);
  }
  {
    // StatusValue<T &>
    MoveCopyTrackerWithInt tracker{NON_DEFAULT_INTEGER};
    StatusValue<MoveCopyTrackerWithInt &> tracker_status_value{tracker};
    const StatusValue<MoveCopyTrackerWithInt &> tracker_status_value_move{
        std::move(tracker_status_value)};
    EXPECT_EQ(tracker_status_value_move.value().num_moves(), 0);
    EXPECT_EQ(tracker_status_value_move.value().num_copies(), 0);
    expect_matches_tracker_value(tracker_status_value_move);
    EXPECT_EQ(&tracker_status_value_move.value(), &tracker);
  }
  {
    // StatusValue<const T &>
    const MoveCopyTrackerWithInt tracker{NON_DEFAULT_INTEGER};
    StatusValue<const MoveCopyTrackerWithInt &> tracker_status_value{tracker};
    const StatusValue<const MoveCopyTrackerWithInt &> tracker_status_value_move{
        std::move(tracker_status_value)};
    EXPECT_EQ(tracker_status_value_move.value().num_moves(), 0);
    EXPECT_EQ(tracker_status_value_move.value().num_copies(), 0);
    expect_matches_tracker_value(tracker_status_value_move);
    EXPECT_EQ(&tracker_status_value_move.value(), &tracker);
  }
  {
    // StatusValue<T &&>
    MoveCopyTrackerWithInt tracker{NON_DEFAULT_INTEGER};
    StatusValue<MoveCopyTrackerWithInt &&> tracker_status_value{
        std::move(tracker)};
    const StatusValue<MoveCopyTrackerWithInt &&> tracker_status_value_move{
        std::move(tracker_status_value)};
    EXPECT_EQ(tracker_status_value_move.value().num_moves(), 0);
    EXPECT_EQ(tracker_status_value_move.value().num_copies(), 0);
    expect_matches_tracker_value(tracker_status_value_move);
  }
  {
    // StatusValue<const T &&>
    MoveCopyTrackerWithInt tracker{NON_DEFAULT_INTEGER};
    StatusValue<const MoveCopyTrackerWithInt &&> tracker_status_value{
        std::move(tracker)};
    const StatusValue<const MoveCopyTrackerWithInt &&>
        tracker_status_value_move{std::move(tracker_status_value)};
    EXPECT_EQ(tracker_status_value_move.value().num_moves(), 0);
    EXPECT_EQ(tracker_status_value_move.value().num_copies(), 0);
    expect_matches_tracker_value(tracker_status_value_move);
  }
}

// Test the rvalue overload of value(). The lvalue overload is tested
// extensively above alongside the constructors.
TEST(StatusValueTest, TestRvalueValue) {
  {
    // StatusValue<T>
    const MoveCopyTrackerWithInt tracker{NON_DEFAULT_INTEGER};
    StatusValue<MoveCopyTrackerWithInt> tracker_status_value{tracker};
    MoveCopyTrackerWithInt &&new_tracker{
        std::move(tracker_status_value).value()};
    EXPECT_EQ(new_tracker.num_moves(), 0);
    EXPECT_EQ(new_tracker.num_copies(), 1);
    EXPECT_EQ(new_tracker.x(), NON_DEFAULT_INTEGER);
  }
  {
    // StatusValue<T &>
    MoveCopyTrackerWithInt tracker{NON_DEFAULT_INTEGER};
    StatusValue<MoveCopyTrackerWithInt &> tracker_status_value{tracker};
    MoveCopyTrackerWithInt &new_tracker{
        std::move(tracker_status_value).value()};
    EXPECT_EQ(new_tracker.num_moves(), 0);
    EXPECT_EQ(new_tracker.num_copies(), 0);
    EXPECT_EQ(new_tracker.x(), NON_DEFAULT_INTEGER);
  }
  {
    // StatusValue<const T &>
    const MoveCopyTrackerWithInt tracker{NON_DEFAULT_INTEGER};
    StatusValue<const MoveCopyTrackerWithInt &> tracker_status_value{tracker};
    const MoveCopyTrackerWithInt &new_tracker{
        std::move(tracker_status_value).value()};
    EXPECT_EQ(new_tracker.num_moves(), 0);
    EXPECT_EQ(new_tracker.num_copies(), 0);
    EXPECT_EQ(new_tracker.x(), NON_DEFAULT_INTEGER);
  }
  {
    // StatusValue<T &&>
    MoveCopyTrackerWithInt tracker{NON_DEFAULT_INTEGER};
    StatusValue<MoveCopyTrackerWithInt &&> tracker_status_value{
        std::move(tracker)};
    MoveCopyTrackerWithInt &&new_tracker{
        std::move(tracker_status_value).value()};
    EXPECT_EQ(new_tracker.num_moves(), 0);
    EXPECT_EQ(new_tracker.num_copies(), 0);
    EXPECT_EQ(new_tracker.x(), NON_DEFAULT_INTEGER);
  }
  {
    // StatusValue<const T &&>
    MoveCopyTrackerWithInt tracker{NON_DEFAULT_INTEGER};
    StatusValue<const MoveCopyTrackerWithInt &&> tracker_status_value{
        std::move(tracker)};
    const MoveCopyTrackerWithInt &&new_tracker{
        std::move(tracker_status_value).value()};
    EXPECT_EQ(new_tracker.num_moves(), 0);
    EXPECT_EQ(new_tracker.num_copies(), 0);
    EXPECT_EQ(new_tracker.x(), NON_DEFAULT_INTEGER);
  }
}

TEST(StatusValueTest, TestConstexpr) {
  {
    constexpr StatusValue<int> FIRST{NON_DEFAULT_INTEGER};
    constexpr StatusValue<int> SECOND = FIRST;
    static_assert(SECOND.ok());
    static_assert(SECOND.value() == NON_DEFAULT_INTEGER);
  }
  {
    constexpr Status TEST_STATUS = MAKE_STATUS("Test Status!");
    constexpr StatusValue<int> FIRST{TEST_STATUS};
    constexpr StatusValue<int> SECOND = FIRST;
    static_assert(not SECOND.ok());
  }
}

// Test RETURN_IF_NOT_OK to make sure it correctly returns when we pass a bad
// status.
TEST(StatusValueTest, TestReturnIfNotOK) {
  const Status test_status = MAKE_STATUS("Test Status!");
  const StatusValue<int> bad_sv{test_status};
  EXPECT_EQ(return_if_not_ok_function(bad_sv).what(), test_status.what());

  const StatusValue<int> good_sv{NON_DEFAULT_INTEGER};
  EXPECT_EQ(return_if_not_ok_function(good_sv).what(), OKAY_STATUS.what());
}

// Test RETURN_OR_ASSIGN with bad statuses
TEST(StatusValueTest, TestReturnOrAssignBadStatus) {
  const Status test_status = MAKE_STATUS("Test Status!");
  {
    const StatusValue<int> bad_sv{test_status};
    EXPECT_EQ(return_or_assign_function(bad_sv).what(), test_status.what());
  }
  {
    const StatusValue<int &> bad_sv{test_status};
    EXPECT_EQ(return_or_assign_function(bad_sv).what(), test_status.what());
  }
  {
    const StatusValue<const int &> bad_sv{test_status};
    EXPECT_EQ(return_or_assign_function(bad_sv).what(), test_status.what());
  }
  {
    StatusValue<int &&> bad_sv{test_status};
    EXPECT_EQ(
        return_or_assign_function(std::move(bad_sv)).what(),
        test_status.what());
  }
  {
    StatusValue<const int &&> bad_sv{test_status};
    EXPECT_EQ(
        return_or_assign_function(std::move(bad_sv)).what(),
        test_status.what());
  }
}

// Test RETURN_OR_ASSIGN with good values
TEST(StatusValueTest, TestReturnOrAssignGoodStatus) {
  {
    const StatusValue<int> good_sv{NON_DEFAULT_INTEGER};
    EXPECT_EQ(return_or_assign_function(good_sv).what(), OKAY_STATUS.what());
  }

  {
    int test_val = NON_DEFAULT_INTEGER;
    const StatusValue<int &> good_sv{test_val};
    EXPECT_EQ(return_or_assign_function(good_sv).what(), OKAY_STATUS.what());
  }
  {
    const StatusValue<const int &> good_sv{NON_DEFAULT_INTEGER};
    EXPECT_EQ(return_or_assign_function(good_sv).what(), OKAY_STATUS.what());
  }
  {
    StatusValue<int &&> good_sv{
        NON_DEFAULT_INTEGER + 0};  // Force an rvalue by adding zero
    EXPECT_EQ(
        return_or_assign_function(std::move(good_sv)).what(),
        OKAY_STATUS.what());
  }
  {
    StatusValue<const int &&> good_sv{
        NON_DEFAULT_INTEGER + 0};  // Force an rvalue by adding zero
    EXPECT_EQ(
        return_or_assign_function(std::move(good_sv)).what(),
        OKAY_STATUS.what());
  }
}

}  // namespace resim
