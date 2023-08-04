// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <cstdint>
#include <functional>

#include "resim/time/timestamp.hh"

namespace resim::time {

// This function finds the minimum number of samples (including both endpoints)
// needed to partition [start_time, end_time] evenly with a step size smaller
// than max_abs_dt. The bounds don't have to be in any particular order. For
// instance, num_samples(0., 3., 2.) = 3 because we'd have a sample at 0, one at
// 1.5, and one at 3.
// @param[in] start_time - One boundary of the interval.
// @param[in] end_time - The other boundary of the interval.
// @param[in] max_abs_dt - The maximum distance between adjacent samples.
// @returns The number of samples.
int64_t num_samples(double start_time, double end_time, double max_abs_dt);

// This function calls a given functor at evenly spaced sample locations on
// [start_time, end_time]. The number of samples is the minimum number of
// samples needed to partition [start_time, end_time] evenly with a step size
// smaller than max_abs_dt. If the start and end time are equal, then the
// functor is called only once at that time.
// @param[in] start_time - One boundary of the interval.
// @param[in] end_time - The other boundary of the interval.
// @param[in] max_abs_dt - The maximum distance between adjacent samples.
// @param[in] func - The functor to call at each sample.
void sample_interval(
    double start_time,
    double end_time,
    double max_abs_dt,
    const std::function<void(double)> &func);

// Overload of num_samples() above that works on Timestamps instead of doubles.
// @param[in] start_time - One boundary of the interval.
// @param[in] end_time - The other boundary of the interval.
// @param[in] max_abs_dt - The maximum distance between adjacent samples.
// @returns The number of samples.
int64_t
num_samples(Timestamp start_time, Timestamp end_time, Duration max_abs_dt);

// Overload of sample_interval() above which works on Timestamps instead of
// doubles. This function guarantees that the functor is called on the closest
// possible Timestamp to the ideal (rational) position. For instance,
// sample_interval(0ns, 8ns, 3ns, func) will call func at (0ns, 3ns, 5ns, 8ns)
// which correspond to rational positions (0/3, 8/3, 16/3, 24/3). Since doubles
// can't represent times larger than 10^7 seconds (~116 days) to nanosecond
// precision, this overload is more accurate when dealing with real-world
// times.
// @param[in] start_time - One boundary of the interval.
// @param[in] end_time - The other boundary of the interval.
// @param[in] max_abs_dt - The maximum distance between adjacent samples.
// @param[in] func - The functor to call at each sample.
void sample_interval(
    Timestamp start_time,
    Timestamp end_time,
    Duration max_abs_dt,
    const std::function<void(Timestamp)> &func);

}  // namespace resim::time
