#include "resim_core/metrics/proto/simple_metric_to_proto.hh"

#include <gtest/gtest.h>

#include <optional>
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
}  // namespace

TEST(SimpleMetricToProtoTests, DoubleRoundTrip) {
  constexpr std::size_t SEED = 694U;
  constexpr double DOUBLE_VALUE = -1837.2;
  std::mt19937 rng(SEED);

  metrics::SimpleMetric simple_metric(
      UUID::new_uuid().to_string(),
      time::Timestamp(time::random_duration(InOut(rng))),
      std::make_optional<double>(DOUBLE_VALUE));

  SimpleMetric simple_metric_proto;
  pack(simple_metric, &simple_metric_proto);

  metrics::SimpleMetric simple_metric_roundtrip = unpack(simple_metric_proto);

  EXPECT_EQ(simple_metric, simple_metric_roundtrip);
}

TEST(SimpleMetricToProtoTests, EmptyRoundTrip) {
  constexpr std::size_t SEED = 694U;
  std::mt19937 rng(SEED);

  metrics::SimpleMetric simple_metric(
      UUID::new_uuid().to_string(),
      time::Timestamp(time::random_duration(InOut(rng))),
      std::nullopt);

  SimpleMetric simple_metric_proto;
  pack(simple_metric, &simple_metric_proto);

  metrics::SimpleMetric simple_metric_roundtrip = unpack(simple_metric_proto);

  EXPECT_EQ(simple_metric, simple_metric_roundtrip);
}

TEST(SimpleMetricToProtoTests, DoublePack) {
  constexpr std::size_t SEED = 39572U;
  constexpr double DOUBLE_VALUE = 103243.312;
  std::mt19937 rng(SEED);
  metrics::SimpleMetric simple_metric(
      UUID::new_uuid().to_string(),
      time::Timestamp(time::random_duration(InOut(rng))),
      std::make_optional<double>(DOUBLE_VALUE));

  SimpleMetric simple_metric_proto;
  pack(simple_metric, &simple_metric_proto);

  EXPECT_EQ(simple_metric_proto.name(), simple_metric.name);
  EXPECT_EQ(
      simple_metric.time,
      time::Timestamp(time::from_seconds_and_nanos(
          {simple_metric_proto.time().seconds(),
           simple_metric_proto.time().nanos()})));

  EXPECT_TRUE(simple_metric_proto.has_metric_value());
  EXPECT_EQ(simple_metric_proto.metric_value(), DOUBLE_VALUE);
}

TEST(SimpleMetricToProtoTests, EmptyPack) {
  constexpr std::size_t SEED = 39572U;
  std::mt19937 rng(SEED);
  metrics::SimpleMetric simple_metric(
      UUID::new_uuid().to_string(),
      time::Timestamp(time::random_duration(InOut(rng))),
      std::nullopt);

  SimpleMetric simple_metric_proto;
  pack(simple_metric, &simple_metric_proto);

  EXPECT_EQ(simple_metric_proto.name(), simple_metric.name);
  EXPECT_EQ(
      simple_metric.time,
      time::Timestamp(time::from_seconds_and_nanos(
          {simple_metric_proto.time().seconds(),
           simple_metric_proto.time().nanos()})));

  EXPECT_FALSE(simple_metric_proto.has_metric_value());
}

}  // namespace resim::metrics::proto
