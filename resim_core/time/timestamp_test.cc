
#include "resim_core/time/timestamp.hh"

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <random>

#include "resim_core/time/random_duration.hh"

namespace resim::time {

using std::literals::chrono_literals::operator""ns;

namespace {
constexpr int NUM_TESTS = 1000;
}  // namespace

TEST(TimestampTest, TestToSecondsAndNanos) {
  // SETUP
  constexpr int SEED = 48;
  std::mt19937 rng{SEED};

  for (int ii = 0; ii < NUM_TESTS; ++ii) {
    const Duration test_duration{random_duration(InOut{rng})};

    // ACTION
    const SecsAndNanos result{to_seconds_and_nanos(test_duration)};

    // VERIFICATION
    EXPECT_EQ(
        result.secs * std::nano::den + result.nanos,
        test_duration.count());
  }
}

TEST(TimestampTest, TestFromSecondsAndNanos) {
  // SETUP
  constexpr int SEED = 956;
  std::mt19937 rng{SEED};
  for (int ii = 0; ii < NUM_TESTS; ++ii) {
    const Duration test_duration{random_duration(InOut{rng})};

    // ACTION
    // to_seconds_and_nanos() is tested above, so we can use it here.
    const SecsAndNanos secs_and_nanos{to_seconds_and_nanos(test_duration)};
    const Duration converted_duration{from_seconds_and_nanos(secs_and_nanos)};

    // VERIFICATION
    EXPECT_EQ(test_duration, converted_duration);
  }
}

TEST(TimestampTest, TestAsSeconds) {
  // SETUP
  constexpr int SEED = 512;
  std::mt19937 rng{SEED};

  for (int ii = 0; ii < NUM_TESTS; ++ii) {
    const Duration test_duration{random_duration(InOut{rng})};

    // ACTION
    const double result_s = as_seconds(test_duration);

    // VERIFICATION
    const Duration result_duration{
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>{result_s})};

    // We need a tolerance here because the conversion to and from
    // double is lossy if we are far from zero
    const Duration tolerance{
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>{
                std::fabs(std::numeric_limits<double>::epsilon() * result_s)})};

    EXPECT_LE(result_duration, test_duration + tolerance);
    EXPECT_GE(result_duration, test_duration - tolerance);
  }
}

TEST(TimestampTest, TestAsDuration) {
  // SETUP
  constexpr int SEED = 512;
  std::mt19937 rng{SEED};

  for (int ii = 0; ii < NUM_TESTS; ++ii) {
    const Duration test_duration{random_duration(InOut{rng})};

    // We can use this since it's tested above:
    const double result_s = as_seconds(test_duration);

    // ACTION
    const Duration result = as_duration(result_s);

    // VERIFICATION
    // We need a tolerance here because the conversion to and from
    // double is lossy if we are far from zero
    const Duration tolerance{
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>{
                std::fabs(std::numeric_limits<double>::epsilon() * result_s)})};

    EXPECT_LE(result, test_duration + tolerance);
    EXPECT_GE(result, test_duration - tolerance);
  }
}

}  // namespace resim::time
