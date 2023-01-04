
#include "resim_core/time/sample_interval.hh"

#include <glog/logging.h>

#include <cmath>

#include "resim_core/math/safe_integer_utils.hh"

namespace resim::time {

int64_t num_samples(double start_time, double end_time, double max_abs_dt) {
  CHECK(max_abs_dt > 0.) << "max_abs_dt must be positive!";
  const double total_time = std::fabs(end_time - start_time);

  // We want to find the smallest integer n such that:
  //
  // max_abs_dt * (n - 1) >= total_time
  //
  // So:
  //
  // n >= total_time / max_abs_dt + 1
  //
  // And we since we know n is an integer:
  //
  // n = ceil(total_time / max_abs_dt) + 1
  //
  return static_cast<int>(std::ceil(total_time / max_abs_dt)) + 1;
}

void sample_interval(
    const double start_time,
    const double end_time,
    const double max_abs_dt,
    const std::function<void(double)> &func) {
  if (start_time == end_time) {
    func(start_time);
    return;
  }
  const int64_t num_steps = num_samples(start_time, end_time, max_abs_dt);
  for (int64_t ii = 0; ii < num_steps; ++ii) {
    const double fraction =
        static_cast<double>(ii) / static_cast<double>(num_steps - 1);
    func((1. - fraction) * start_time + fraction * end_time);
  }
}

int64_t num_samples(
    const Timestamp start_time,
    const Timestamp end_time,
    const Duration max_abs_dt) {
  CHECK(max_abs_dt.count() > 0) << "max_abs_dt must be positive!";
  const Duration total_time{math::safe_abs(math::safe_difference(
      end_time.time_since_epoch().count(),
      start_time.time_since_epoch().count()))};

  // We want to find the smallest integer n such that:
  //
  // max_abs_dt * (n - 1) >= total_time
  //
  // Since this is an integer equation (with things wrapped in Durations), this
  // is equivalent to:
  //
  // max_abs_dt * (n - 1) > total_time - 1
  //
  // So:
  //
  // n > (total_time - 1) / max_abs_dt + 1
  //
  // So we can define:
  //
  // n = (total_time - 1) / max_abs_dt + 2
  //
  // This sum could oveflow if max_abs_dt.count() == 1 and total_time.count() ==
  // INT64_MAX
  return math::safe_sum((total_time.count() - 1) / max_abs_dt.count(), 2);
}

void sample_interval(
    const Timestamp start_time,
    const Timestamp end_time,
    const Duration max_abs_dt,
    const std::function<void(Timestamp)> &func) {
  if (start_time == end_time) {
    func(start_time);
    return;
  }
  const int64_t samples = num_samples(start_time, end_time, max_abs_dt);
  const int64_t intervals = samples - 1;
  const Duration total_time{math::safe_difference(
      end_time.time_since_epoch().count(),
      start_time.time_since_epoch().count())};

  // When we divide the range into perfect intervals, we get rational
  // samples. We track the locations of these rational samples using a
  // mixed number so that we don't risk integer overflow in our
  // numerator. In particular, every rational sample position is given
  // by:
  //
  // whole + numerator / intervals
  //
  // Since we need to make a timestamp with an integer count, we
  // always just round to the nearest integer.
  int64_t whole = 0;
  int64_t numerator = 0;
  for (int64_t ii = 0; ii < samples; ++ii) {
    // Should never overflow since rounded will be less than or equal to
    // total_time.
    const int64_t rounded = whole + ((2 * numerator >= intervals) ? 1 : 0);
    const Duration time{rounded};

    func(start_time + time);

    // Increment
    numerator = math::safe_sum(numerator, total_time.count());

    // Restore the mixed number
    whole += numerator / intervals;
    numerator %= intervals;
  }
}

}  // namespace resim::time
