#include "resim/time/sample_interval.hh"

#include <gtest/gtest.h>

#include <chrono>
#include <iomanip>
#include <random>

namespace resim::time {

using std::literals::chrono_literals::operator""ns;

class SampleIntervalTest : public ::testing::Test {
 protected:
  static constexpr unsigned SEED = 89U;

  static constexpr int INT_LB = 1;
  static constexpr int INT_UB = 100;

  static constexpr double DOUBLE_LB = 0.1;
  static constexpr double DOUBLE_UB = 100.0;

  std::mt19937 rng_{SEED};
  std::uniform_int_distribution<int> int_dist_{INT_LB, INT_UB};
  std::uniform_real_distribution<double> double_dist_{DOUBLE_LB, DOUBLE_UB};
};

TEST_F(SampleIntervalTest, TestNumSamplesDouble) {
  constexpr int NUM_TESTS = 200;
  for (int ii = 0; ii < NUM_TESTS; ++ii) {
    // SETUP
    const double start_time = double_dist_(rng_);
    const double end_time = double_dist_(rng_);
    const double period = end_time - start_time;
    constexpr double EPSILON = 1e-7;
    const int divisor = int_dist_(rng_);

    // ACTION / VERIFICATION
    {
      // We should get divisor + 1 samples in this case since max_abs_dt is
      // just barely greater than period/divisor.
      const double max_abs_dt = std::fabs(period / divisor) + EPSILON;
      EXPECT_EQ(num_samples(start_time, end_time, max_abs_dt), divisor + 1);
    }
    {
      // We should have to take an extra step here compared to the above since
      // we must choose a dt smaller than period/divisor.
      const double max_abs_dt = std::fabs(period / divisor) - EPSILON;
      EXPECT_EQ(num_samples(start_time, end_time, max_abs_dt), divisor + 2);
    }
  }
}

TEST_F(SampleIntervalTest, TestSampleIntervalDouble) {
  constexpr int NUM_TESTS = 200;
  for (int ii = 0; ii < NUM_TESTS; ++ii) {
    // SETUP
    const double start_time = double_dist_(rng_);
    const double end_time = double_dist_(rng_);
    const double period = end_time - start_time;
    constexpr double EPSILON = 1e-8;
    const int divisor = int_dist_(rng_);
    const double max_abs_dt = std::fabs(period / divisor);

    double expected_time = start_time;
    const double expected_dt =
        period /
        static_cast<double>(num_samples(start_time, end_time, max_abs_dt) - 1);

    // ACTION / VERIFICATION
    sample_interval(start_time, end_time, max_abs_dt, [&](const double time) {
      EXPECT_NEAR(expected_time, time, EPSILON);
      expected_time += expected_dt;
    });
  }
}

TEST_F(SampleIntervalTest, TestNumSamplesTimestamp) {
  constexpr int NUM_TESTS = 200;
  for (int ii = 0; ii < NUM_TESTS; ++ii) {
    // SETUP
    const Timestamp start_time{as_duration(double_dist_(rng_))};
    const Timestamp end_time{as_duration(double_dist_(rng_))};
    const Duration period = end_time - start_time;
    const int expected_num_intervals = int_dist_(rng_);

    // ACTION / VERIFICATION
    {
      // If the max_abs_dt is barely more than the period divided into
      // intervals, we should get back the number of intervals plus
      // one for the total sample amount.
      const Duration max_abs_dt{
          std::abs(period.count() / expected_num_intervals) + 1};

      EXPECT_EQ(
          num_samples(start_time, end_time, max_abs_dt),
          expected_num_intervals + 1);
    }
    {
      // If max_abs_dt is barely less than the period divided into
      // intervals, we should require one more interval than in the
      // above case.
      const Duration max_abs_dt{
          std::abs(period.count() / expected_num_intervals) - 1};

      // This could theoretically happen although it is vanishingly
      // unlikely and so we won't make the test more complex to
      // explicitly avoid it.
      ASSERT_GT(max_abs_dt, 0ns);

      EXPECT_EQ(
          num_samples(start_time, end_time, max_abs_dt),
          expected_num_intervals + 2);
    }
  }
}

TEST_F(SampleIntervalTest, TestSampleIntervalTimestamp) {
  constexpr int NUM_TESTS = 1;
  for (int ii = 0; ii < NUM_TESTS; ++ii) {
    // SETUP
    const Timestamp start_time{as_duration(double_dist_(rng_))};
    const Timestamp end_time{as_duration(double_dist_(rng_))};
    const Duration period = end_time - start_time;
    const int num_intervals = int_dist_(rng_);
    const Duration max_abs_dt{std::abs(period.count() / num_intervals) + 1};

    // ACTION / VERIFICATION
    int jj = 0;
    sample_interval(
        start_time,
        end_time,
        max_abs_dt,
        [&](const Timestamp time) {
          const double fraction = static_cast<double>(jj) / num_intervals;
          const double time_s = fraction * as_seconds(end_time - start_time);
          // Just more than half a nano to account for rounding.
          constexpr double EPSILON = 5.1e-10;
          EXPECT_NEAR(time_s, as_seconds(time - start_time), EPSILON);
          ++jj;
        });
  }
}

TEST_F(SampleIntervalTest, TestSampleZeroIntervalDouble) {
  constexpr double START_TIME = 0.;
  constexpr double END_TIME = START_TIME;
  constexpr double MAX_ABS_DT = 1e-3;

  int jj = 0;
  sample_interval(START_TIME, END_TIME, MAX_ABS_DT, [&](const double time) {
    EXPECT_EQ(START_TIME, time);
    ++jj;
  });
  constexpr int NUM_SAMPLES = 1;
  EXPECT_EQ(jj, NUM_SAMPLES);
}

TEST_F(SampleIntervalTest, TestSampleZeroIntervalTimestamp) {
  constexpr Timestamp START_TIME{0ns};
  constexpr Timestamp END_TIME{START_TIME};
  constexpr Duration MAX_ABS_DT{1ns};

  int jj = 0;
  sample_interval(START_TIME, END_TIME, MAX_ABS_DT, [&](const Timestamp time) {
    EXPECT_EQ(START_TIME, time);
    ++jj;
  });
  constexpr int NUM_SAMPLES = 1;
  EXPECT_EQ(jj, NUM_SAMPLES);
}

}  // namespace resim::time
