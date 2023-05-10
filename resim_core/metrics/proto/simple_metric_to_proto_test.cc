#include "resim_core/metrics/proto/simple_metric_to_proto.hh"

#include <gtest/gtest.h>

#include <random>
#include <vector>

#include "resim_core/metrics/proto/simple_metric.pb.h"
#include "resim_core/metrics/simple_metric.hh"
#include "resim_core/time/random_duration.hh"
#include "resim_core/time/timestamp.hh"
#include "resim_core/utils/uuid.hh"

namespace resim::metrics::proto {

namespace {
using time::Timestamp;
template <typename Rng>
std::vector<double> random_double_vector(Rng&& rng) {
  constexpr double RAND_MIN_MAX =
      1000.0;  // Better *not* to use numeric max/lowest here, to reduce chance
               // of numerical error
  constexpr std::size_t MIN_LENGTH = 2;
  constexpr std::size_t MAX_LENGTH = 20;
  std::uniform_int_distribution<size_t> length_distr(MIN_LENGTH, MAX_LENGTH);

  std::uniform_real_distribution<double> value_distr(
      -RAND_MIN_MAX,
      RAND_MIN_MAX);
  std::size_t length = length_distr(rng);
  std::vector<double> vec(length, 0.0);

  for (std::size_t i = 0; i < length; ++i) {
    vec[i] = value_distr(rng);
  }

  return vec;
}
}  // namespace

TEST(SimpleMetricToProtoTests, RandomRoundTrip) {
  constexpr std::size_t SEED = 694U;
  std::mt19937 rng(SEED);

  metrics::SimpleMetric simple_metric(
      UUID::new_uuid().to_string(),
      time::Timestamp(time::random_duration(InOut(rng))),
      random_double_vector(rng));

  SimpleMetric simple_metric_proto;
  pack(simple_metric, &simple_metric_proto);

  metrics::SimpleMetric simple_metric_roundtrip = unpack(simple_metric_proto);

  EXPECT_EQ(simple_metric, simple_metric_roundtrip);
}

TEST(SimpleMetricToProtoTests, RandomPack) {
  constexpr std::size_t SEED = 39572U;
  std::mt19937 rng(SEED);
  metrics::SimpleMetric simple_metric(
      UUID::new_uuid().to_string(),
      time::Timestamp(time::random_duration(InOut(rng))),
      random_double_vector(rng));

  SimpleMetric simple_metric_proto;
  pack(simple_metric, &simple_metric_proto);

  EXPECT_EQ(simple_metric_proto.name(), simple_metric.name);
  EXPECT_EQ(
      simple_metric.time,
      time::Timestamp(time::from_seconds_and_nanos(
          {simple_metric_proto.time().seconds(),
           simple_metric_proto.time().nanos()})));

  for (std::size_t i = 0; i < simple_metric.value.size(); ++i) {
    EXPECT_EQ(simple_metric.value[i], simple_metric_proto.value(i));
  }
}

TEST(SimpleMetricToProtoTests, EmptyRoundTrip) {
  metrics::SimpleMetric simple_metric{};

  SimpleMetric simple_metric_proto;
  pack(simple_metric, &simple_metric_proto);

  metrics::SimpleMetric simple_metric_roundtrip = unpack(simple_metric_proto);

  EXPECT_EQ(simple_metric, simple_metric_roundtrip);
}
}  // namespace resim::metrics::proto
