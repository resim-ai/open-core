# Time

## Basic Types

Within the ReSim libraries, we generally represent durations using
`resim::time::Duration`, which is an alias for `std::chrono::nanoseconds` and
timestamps as `resim::time::Timestamp` which is an alias for
`std::chrono::sys_time<resim::time::Duration>`. 


## Converters


### Floating Point Representation

While we generally use the aforementioned integer-based timestamps and
durations where possible, it is very often convenient to store times as
double-precision floating point values, especially when using them in physical
computations. If the values are sufficiently small (e.g. elapsed time in a
sim), this can be done with no loss of accuracy. To facilitate, this, we have
convenience converters:

```
#include "resim/time/timestamp.hh"
#include "resim/assert/assert.hh"

// ...

using namespace resim::time;
Duration my_duration{std::chrono::nanoseconds(10000U)};

double my_duration_s = as_seconds(my_duration);

// Should pass since my_duration is small
REASSERT(as_duration(my_duration_s) == my_duration);

my_duration += std::chrono::system_clock::now().time_since_epoch();

my_duration_s = as_seconds(my_duration);

// Will likely not pass because my_duration is big and precision is lost
// converting it to a double.
REASSERT(as_duration(my_duration_s) == my_duration);
```

### Seconds & Nanoseconds Struct Representation

It's common in time serialization formats (e.g.
[google::protobuf::Timestamp](https://github.com/protocolbuffers/protobuf/blob/main/src/google/protobuf/timestamp.proto)
and ROS2's
[builtin_interfaces/msg/Time.msg](https://github.com/ros2/rcl_interfaces/blob/master/builtin_interfaces/msg/Time.msg))
for the seconds and nanoseconds to be stored as separate integer counts. To
facilitate conversion to and from such serialization types, we have a time
representation called `SecsAndNanos`:

```
struct SecsAndNanos {
  int64_t secs = 0;
  int32_t nanos = 0;
};
```

And converters to and from it:

```
const SecsAndNanos my_secs_and_nanos = to_seconds_and_nanos(my_duration);
REASSERT(from_seconds_and_nanos(my_secs_and_nanos) == my_duration);
```

## Event Scheduling

Running simulations or processing logs often requires iterating through an
ordered set of times in increasing order. These sets of interesting timestamps
are not always uniformly spaced, and sometimes we want to add future times to
this set as we are iterating through it. For instance, when modeling message
transmission latency in a simulated system (i.e. one where we are explicitly
controlling a simulated time), we may want to schedule a future time when we
expect the message to arrive so we can properly transmit it to its receiver and
keep the order of messages consistent with the real world system. To support
this functionality, we define the `EventSchedule` class template which can
store a time-ordered queue of events with arbitrary payloads. Here's a simple
example of how to use it:

```
#include <iostream>

#include "resim/time/event_schedule.hh"

// ...

EventSchedule<std::string> my_messages;
my_messages.schedule_event(Timestamp(std::chrono::nanoseconds(200)), "ReSim");
my_messages.schedule_event(Timestamp(std::chrono::nanoseconds(100)), "Hello");
my_messages.schedule_event(Timestamp(std::chrono::nanoseconds(200)), "user!");

std::cout.precision(9);
std::cout << std::fixed;
while (my_messages.size() > 0U) {
  const auto event = my_messages.top_event();
  std::cout << "[" << event.time.time_since_epoch().count() / 1e9 << "] "
            << event.payload << std::endl;
  my_messages.pop_event();
}
```

Note that the event schedule is "stable" in the sense that it's
first-in-first-out for events with the same timestamp. Hence the code above will print out:

```
[0.000000100] Hello
[0.000000200] ReSim
[0.000000200] user!
```

## Interval Sampling

Another common operation in running simulations and collecing log metrics is
sampling an interval uniformly. For example, one may want to numerically
compute a robot's average deviation from its desired pose over the course of a
simulation. If a user wants to compute this average with Reimann rectangles,
they could pick a maximum rectangle width (dt) that they are willing to accept
(depending on what accuracy they desire) and then compute the integral using
`resim::time::sample_interval()` and the associated
`resim::time::num_samples()` function like so:

```
#include <cmath>

#include "resim/time/sample_interval.hh"

// ...

const Timestamp start_time;
const Timestamp end_time{start_time + std::chrono::seconds(30)};
const Duration max_dt = std::chrono::microseconds(100);

// A stand in function for illustration purposes.
const auto deviation = [](const Timestamp &t) {
  return std::cos(std::sqrt(as_seconds(t.time_since_epoch())));
};

double integral = 0.;
sample_interval(start_time, end_time, max_dt, [&](const Timestamp &t) {
  // Left rectangles so we leave off the value at the end time
  if (t != end_time) {
    integral += deviation(t);
  }
});

// The actual dt per rectangle is the total interval divided by (N - 1)
const int N = num_samples(start_time, end_time, max_dt);
const double dt_s = as_seconds(end_time - start_time) / (N - 1);
integral *= dt_s;

// Make sure we match the analytical result
const double analytical_result = 2. * (-1. + std::cos(std::sqrt(30.)) +
                                       std::sqrt(30.) * std::sin(std::sqrt(30.)));
REASSERT(std::fabs(integral - analytical_result) < 1e-4);
```

!!! Note
    Feel free to play around with the [source
    code](https://github.com/resim-ai/re-core/blob/main/resim/examples/time.cc)
    for the examples above.
