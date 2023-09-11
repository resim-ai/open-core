// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/metrics/proto/validate_metrics_proto.hh"

#include <google/protobuf/util/json_util.h>
#include <gtest/gtest.h>

#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include "resim/assert/assert.hh"
#include "resim/metrics/proto/metrics.pb.h"
#include "resim/time/proto/time_to_proto.hh"
#include "resim/time/timestamp.hh"
#include "resim/utils/proto/uuid_to_proto.hh"
#include "resim/utils/uuid.hh"
#include "tools/cpp/runfiles/runfiles.h"

namespace resim::metrics::proto {

namespace {

class MetricsProtoTest : public ::testing::Test {
 public:
  static std::unique_ptr<bazel::tools::cpp::runfiles::Runfiles>
      runfiles;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

 protected:
  static void SetUpTestSuite() {
    std::string error;
    runfiles = std::unique_ptr<bazel::tools::cpp::runfiles::Runfiles>(
        bazel::tools::cpp::runfiles::Runfiles::CreateForTest(&error));
    REASSERT(runfiles != nullptr, error);
  }

  static void TearDownTestSuite() {
    google::protobuf::ShutdownProtobufLibrary();
  }
};

std::unique_ptr<bazel::tools::cpp::runfiles::Runfiles> MetricsProtoTest::
    runfiles;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

constexpr double ONE = 1.0;
constexpr double TWO = 2.0;
constexpr int INVALID_ENUM = 100000;
}  // namespace

JobMetrics read_metrics_json(const std::string& metrics_path) {
  std::string metrics_file_path =
      MetricsProtoTest::runfiles->Rlocation(metrics_path);
  REASSERT(
      !metrics_file_path.empty(),
      "Failed to find metrics.json in runfiles.");

  std::ifstream file(metrics_file_path);
  std::stringstream buffer;
  buffer << file.rdbuf();

  JobMetrics message;
  google::protobuf::util::JsonParseOptions options;
  auto status = google::protobuf::util::JsonStringToMessage(
      buffer.str(),
      &message,
      options);

  REASSERT(status.ok(), "Failed to convert metrics.json to protobuf");

  return message;
}

TEST_F(MetricsProtoTest, TestValidProto) {
  JobMetrics metrics =
      read_metrics_json("resim_open_core/resim/metrics/proto/metrics.json");
  validate_job_metrics_proto(metrics);
}

TEST_F(MetricsProtoTest, TestInvalidMetricsDataType) {
  JobMetrics job_metrics;
  UUID job_id = UUID::new_uuid();
  resim::proto::pack(job_id, job_metrics.mutable_job_id()->mutable_job_id());

  // Make some string data
  auto* string_data = job_metrics.add_metrics_data();
  UUID string_data_id = UUID::new_uuid();
  resim::proto::pack(
      string_data_id,
      string_data->mutable_id()->mutable_data_id());
  string_data->set_data_type(MetricsDataType::STRING_ARRAY_DATA_TYPE);
  string_data->set_length(2);
  string_data->set_name("Strings");
  string_data->set_is_per_actor(false);
  string_data->mutable_array()->mutable_strings()->add_strings("String 1");
  string_data->mutable_array()->mutable_strings()->add_strings("String 2");

  // Test string data validates when type is correct
  validate_metrics_data_proto(
      *string_data,
      build_metrics_data_map(job_metrics.metrics_data()));

  // Make some double data
  auto* double_data = job_metrics.add_metrics_data();
  UUID double_data_id = UUID::new_uuid();
  resim::proto::pack(
      double_data_id,
      double_data->mutable_id()->mutable_data_id());
  double_data->set_data_type(MetricsDataType::DOUBLE_ARRAY_DATA_TYPE);
  double_data->set_length(2);
  double_data->set_name("Doubles");
  double_data->set_is_per_actor(false);
  double_data->mutable_array()->mutable_doubles()->add_doubles(ONE);
  double_data->mutable_array()->mutable_doubles()->add_doubles(TWO);

  // Test double data validates when type is correct
  validate_metrics_data_proto(
      *double_data,
      build_metrics_data_map(job_metrics.metrics_data()));

  // Make some status data
  auto* status_data = job_metrics.add_metrics_data();
  UUID status_data_id = UUID::new_uuid();
  resim::proto::pack(
      status_data_id,
      status_data->mutable_id()->mutable_data_id());
  status_data->set_data_type(MetricsDataType::METRIC_STATUS_ARRAY_DATA_TYPE);
  status_data->set_length(2);
  status_data->set_name("Statuses");
  status_data->set_is_per_actor(false);
  status_data->mutable_array()->mutable_statuses()->add_statuses(
      MetricStatus::PASSED);
  status_data->mutable_array()->mutable_statuses()->add_statuses(
      MetricStatus::PASSED);

  // Should fail when types don't match
  string_data->set_data_type(MetricsDataType::ACTOR_ID_ARRAY_DATA_TYPE);
  EXPECT_THROW(
      validate_metrics_data_proto(
          *string_data,
          build_metrics_data_map(job_metrics.metrics_data())),
      AssertException);
  string_data->set_data_type(MetricsDataType::STRING_ARRAY_DATA_TYPE);

  // Make a bar chart metric
  auto* metric = job_metrics.mutable_job_metrics()->add_metrics();
  UUID metric_id = UUID::new_uuid();

  resim::proto::pack(metric_id, metric->mutable_id()->mutable_metric_id());
  metric->set_type(MetricType::BAR_CHART);
  metric->set_status(MetricStatus::PASSED);
  resim::proto::pack(job_id, metric->mutable_job_id()->mutable_job_id());

  auto* metric_values =
      metric->mutable_metric_values()->mutable_bar_chart_metric_values();

  auto* doubles_value_id =
      metric_values->add_values_data_id()->mutable_data_id();
  resim::proto::pack(double_data_id, doubles_value_id);
  resim::proto::pack(
      status_data_id,
      metric_values->add_statuses_data_id()->mutable_data_id());

  // Should validate when data has double type
  validate_metric_proto(
      *metric,
      build_metrics_data_map(job_metrics.metrics_data()));

  // Should fail when data has string type - even though string data itself is
  // valid.
  resim::proto::pack(string_data_id, doubles_value_id);
  EXPECT_THROW(
      validate_metric_proto(
          *metric,
          build_metrics_data_map(job_metrics.metrics_data())),
      AssertException);
}

TEST_F(MetricsProtoTest, RepeatDataIDs) {
  JobMetrics job_metrics;
  UUID job_id = UUID::new_uuid();
  job_metrics.set_metrics_status(MetricStatus::PASSED);
  resim::proto::pack(job_id, job_metrics.mutable_job_id()->mutable_job_id());
  job_metrics.mutable_job_metrics()->set_metrics_status(MetricStatus::PASSED);

  // Make some timestamp data
  auto* timestamp_data = job_metrics.add_metrics_data();
  UUID timestamp_data_id = UUID::new_uuid();
  resim::proto::pack(
      timestamp_data_id,
      timestamp_data->mutable_id()->mutable_data_id());
  timestamp_data->set_data_type(MetricsDataType::TIMESTAMP_ARRAY_DATA_TYPE);
  timestamp_data->set_length(2);
  timestamp_data->set_name("Timestamps");
  timestamp_data->set_is_per_actor(false);
  resim::time::proto::pack(
      time::Timestamp(std::chrono::seconds(0)),
      timestamp_data->mutable_array()->mutable_timestamps()->add_timestamps());
  resim::time::proto::pack(
      time::Timestamp(std::chrono::seconds(1)),
      timestamp_data->mutable_array()->mutable_timestamps()->add_timestamps());
  validate_metrics_data_proto(
      *timestamp_data,
      build_metrics_data_map(job_metrics.metrics_data()));

  // Make some indexed timestamp data
  auto* indexed_timestamp_data = job_metrics.add_metrics_data();
  UUID indexed_timestamp_data_id = UUID::new_uuid();
  resim::proto::pack(
      indexed_timestamp_data_id,
      indexed_timestamp_data->mutable_id()->mutable_data_id());
  indexed_timestamp_data->set_data_type(
      MetricsDataType::INDEXED_TIMESTAMP_ARRAY_DATA_TYPE);
  indexed_timestamp_data->set_length(2);
  indexed_timestamp_data->set_name("Indexed timestamps");
  indexed_timestamp_data->set_is_per_actor(false);
  resim::time::proto::pack(
      time::Timestamp(std::chrono::seconds(2)),
      indexed_timestamp_data->mutable_array()
          ->mutable_indexed_timestamps()
          ->add_timestamps());
  resim::time::proto::pack(
      time::Timestamp(std::chrono::seconds(3)),
      indexed_timestamp_data->mutable_array()
          ->mutable_indexed_timestamps()
          ->add_timestamps());
  resim::proto::pack(
      timestamp_data_id,
      indexed_timestamp_data->mutable_array()
          ->mutable_indexed_timestamps()
          ->mutable_index_id()
          ->mutable_data_id());
  indexed_timestamp_data->mutable_array()
      ->mutable_indexed_timestamps()
      ->set_index_type(MetricsDataType::TIMESTAMP_ARRAY_DATA_TYPE);
  validate_metrics_data_proto(
      *indexed_timestamp_data,
      build_metrics_data_map(job_metrics.metrics_data()));

  // Make some indexed actor ID data
  auto* indexed_actor_id_data = job_metrics.add_metrics_data();
  UUID indexed_actor_id_data_id = UUID::new_uuid();
  resim::proto::pack(
      indexed_actor_id_data_id,
      indexed_actor_id_data->mutable_id()->mutable_data_id());
  indexed_actor_id_data->set_data_type(
      MetricsDataType::INDEXED_ACTOR_ID_ARRAY_DATA_TYPE);
  indexed_actor_id_data->set_length(2);
  indexed_actor_id_data->set_name("Indexed Actor IDs");
  indexed_actor_id_data->set_is_per_actor(false);
  std::cout << indexed_actor_id_data->length() << std::endl;
  resim::proto::pack(
      UUID::new_uuid(),
      indexed_actor_id_data->mutable_array()
          ->mutable_indexed_actor_ids()
          ->add_actor_ids()
          ->mutable_actor_id());
  resim::proto::pack(
      UUID::new_uuid(),
      indexed_actor_id_data->mutable_array()
          ->mutable_indexed_actor_ids()
          ->add_actor_ids()
          ->mutable_actor_id());
  resim::proto::pack(
      timestamp_data_id,
      indexed_actor_id_data->mutable_array()
          ->mutable_indexed_actor_ids()
          ->mutable_index_id()
          ->mutable_data_id());
  indexed_actor_id_data->mutable_array()
      ->mutable_indexed_actor_ids()
      ->set_index_type(MetricsDataType::TIMESTAMP_ARRAY_DATA_TYPE);

  validate_metrics_data_proto(
      *indexed_actor_id_data,
      build_metrics_data_map(job_metrics.metrics_data()));

  validate_job_metrics_proto(job_metrics);

  std::unordered_map<resim::UUID, MetricsData> old_data_map =
      build_metrics_data_map(job_metrics.metrics_data());

  // Duplicate an ID
  resim::proto::pack(
      indexed_actor_id_data_id,
      indexed_timestamp_data->mutable_id()->mutable_data_id());

  // Should fail due to duplicate IDs, regardless of data map used.
  EXPECT_THROW(validate_job_metrics_proto(job_metrics), AssertException);
  EXPECT_THROW(
      validate_job_metrics_proto(job_metrics, old_data_map),
      AssertException);
}

TEST_F(MetricsProtoTest, InvalidDataTypes) {
  JobMetrics job_metrics;
  UUID job_id = UUID::new_uuid();
  job_metrics.set_metrics_status(MetricStatus::PASSED);
  resim::proto::pack(job_id, job_metrics.mutable_job_id()->mutable_job_id());
  job_metrics.mutable_job_metrics()->set_metrics_status(MetricStatus::PASSED);

  // Make some timestamp data
  auto* timestamp_data = job_metrics.add_metrics_data();
  UUID timestamp_data_id = UUID::new_uuid();
  resim::proto::pack(
      timestamp_data_id,
      timestamp_data->mutable_id()->mutable_data_id());
  timestamp_data->set_data_type(MetricsDataType::TIMESTAMP_ARRAY_DATA_TYPE);

  timestamp_data->set_length(2);
  timestamp_data->set_name("Timestamps");
  timestamp_data->set_is_per_actor(false);
  resim::time::proto::pack(
      time::Timestamp(std::chrono::seconds(0)),
      timestamp_data->mutable_array()->mutable_timestamps()->add_timestamps());
  resim::time::proto::pack(
      time::Timestamp(std::chrono::seconds(1)),
      timestamp_data->mutable_array()->mutable_timestamps()->add_timestamps());

  // Should validate with correct types
  validate_metrics_data_proto(
      *timestamp_data,
      build_metrics_data_map(job_metrics.metrics_data()));

  // Should not validate with another type
  timestamp_data->set_data_type(MetricsDataType::DOUBLE_ARRAY_DATA_TYPE);
  EXPECT_THROW(
      validate_metrics_data_proto(
          *timestamp_data,
          build_metrics_data_map(job_metrics.metrics_data())),
      AssertException);

  // Should not validate with no type
  timestamp_data->set_data_type(MetricsDataType::NO_DATA_TYPE);
  EXPECT_THROW(
      validate_metrics_data_proto(
          *timestamp_data,
          build_metrics_data_map(job_metrics.metrics_data())),
      AssertException);

  // Should not validate with invalid type
  timestamp_data->set_data_type(static_cast<MetricsDataType>(INVALID_ENUM));
  EXPECT_THROW(
      validate_metrics_data_proto(
          *timestamp_data,
          build_metrics_data_map(job_metrics.metrics_data())),
      AssertException);
}

TEST_F(MetricsProtoTest, InvalidMetricTypes) {
  JobMetrics job_metrics;
  UUID job_id = UUID::new_uuid();
  job_metrics.set_metrics_status(MetricStatus::FAILED);
  resim::proto::pack(job_id, job_metrics.mutable_job_id()->mutable_job_id());
  job_metrics.mutable_job_metrics()->set_metrics_status(MetricStatus::FAILED);

  // Make some double data
  auto* double_data = job_metrics.add_metrics_data();
  UUID double_data_id = UUID::new_uuid();
  resim::proto::pack(
      double_data_id,
      double_data->mutable_id()->mutable_data_id());
  double_data->set_data_type(MetricsDataType::DOUBLE_ARRAY_DATA_TYPE);
  double_data->set_length(1);
  double_data->set_name("Doubles");
  double_data->set_is_per_actor(false);
  double_data->mutable_array()->mutable_doubles()->add_doubles(ONE);

  // Make some status data
  auto* status_data = job_metrics.add_metrics_data();
  UUID status_data_id = UUID::new_uuid();
  resim::proto::pack(
      status_data_id,
      status_data->mutable_id()->mutable_data_id());
  status_data->set_data_type(MetricsDataType::METRIC_STATUS_ARRAY_DATA_TYPE);
  status_data->set_length(1);
  status_data->set_name("Statuses");
  status_data->set_is_per_actor(false);
  status_data->mutable_array()->mutable_statuses()->add_statuses(
      MetricStatus::FAILED);

  // Make a double summary metric
  auto* metric = job_metrics.mutable_job_metrics()->add_metrics();
  UUID metric_id = UUID::new_uuid();

  resim::proto::pack(metric_id, metric->mutable_id()->mutable_metric_id());
  metric->set_type(MetricType::DOUBLE_SUMMARY);
  metric->set_status(MetricStatus::PASSED);
  resim::proto::pack(job_id, metric->mutable_job_id()->mutable_job_id());

  auto* metric_values =
      metric->mutable_metric_values()->mutable_double_metric_values();

  resim::proto::pack(
      double_data_id,
      metric_values->mutable_value_data_id()->mutable_data_id());
  resim::proto::pack(
      status_data_id,
      metric_values->mutable_status_data_id()->mutable_data_id());

  // Should validate with valid type
  validate_job_metrics_proto(job_metrics);

  // Should not validate with another type
  metric->set_type(MetricType::BAR_CHART);
  EXPECT_THROW(validate_job_metrics_proto(job_metrics), AssertException);

  // Should not validate with no type
  metric->set_type(MetricType::NO_METRIC_TYPE);
  EXPECT_THROW(validate_job_metrics_proto(job_metrics), AssertException);

  // Should not validate with an invalid type
  metric->set_type(static_cast<MetricType>(INVALID_ENUM));
  EXPECT_THROW(validate_job_metrics_proto(job_metrics), AssertException);
}

TEST_F(MetricsProtoTest, ValidatePerActorData) {
  JobMetrics job_metrics;
  UUID job_id = UUID::new_uuid();
  job_metrics.set_metrics_status(MetricStatus::PASSED);
  resim::proto::pack(job_id, job_metrics.mutable_job_id()->mutable_job_id());
  job_metrics.mutable_job_metrics()->set_metrics_status(MetricStatus::PASSED);

  // Make some double data
  auto* double_data = job_metrics.add_metrics_data();
  UUID double_data_id = UUID::new_uuid();
  resim::proto::pack(
      double_data_id,
      double_data->mutable_id()->mutable_data_id());
  double_data->set_data_type(MetricsDataType::DOUBLE_ARRAY_DATA_TYPE);
  double_data->set_length(1);
  double_data->set_name("Doubles per actor");
  double_data->set_is_per_actor(true);
  UUID actor_one_id = UUID::new_uuid();
  UUID actor_two_id = UUID::new_uuid();

  resim::proto::pack(
      actor_one_id,
      double_data->add_actor_ids()->mutable_actor_id());
  resim::proto::pack(
      actor_two_id,
      double_data->add_actor_ids()->mutable_actor_id());

  auto* actor_one_data =
      double_data->mutable_per_actor_data()->add_actor_data();
  resim::proto::pack(
      actor_one_id,
      actor_one_data->mutable_actor_id()->mutable_actor_id());
  resim::proto::pack(
      double_data_id,
      actor_one_data->mutable_parent_id()->mutable_data_id());
  actor_one_data->set_length(1);
  actor_one_data->mutable_array()->mutable_doubles()->add_doubles(ONE);

  auto* actor_two_data =
      double_data->mutable_per_actor_data()->add_actor_data();
  resim::proto::pack(
      actor_two_id,
      actor_two_data->mutable_actor_id()->mutable_actor_id());
  resim::proto::pack(
      double_data_id,
      actor_two_data->mutable_parent_id()->mutable_data_id());
  actor_two_data->set_length(1);
  actor_two_data->mutable_array()->mutable_doubles()->add_doubles(TWO);

  // Per actor data should validate
  resim::metrics::proto::validate_metrics_data_proto(
      *double_data,
      resim::metrics::proto::build_metrics_data_map(
          job_metrics.metrics_data()));

  // Should not validate if an invalid actor ID is added
  resim::proto::pack(
      UUID::new_uuid(),
      actor_one_data->mutable_actor_id()->mutable_actor_id());
  EXPECT_THROW(
      resim::metrics::proto::validate_metrics_data_proto(
          *double_data,
          resim::metrics::proto::build_metrics_data_map(
              job_metrics.metrics_data())),
      AssertException);
  resim::proto::pack(
      actor_one_id,
      actor_one_data->mutable_actor_id()->mutable_actor_id());

  // Should not validate if an invalid length is passed
  double_data->set_length(2);
  EXPECT_THROW(
      resim::metrics::proto::validate_metrics_data_proto(
          *double_data,
          resim::metrics::proto::build_metrics_data_map(
              job_metrics.metrics_data())),
      AssertException);
  double_data->set_length(1);

  // Should not validate if an invalid parent ID is passed
  resim::proto::pack(
      UUID::new_uuid(),
      actor_one_data->mutable_parent_id()->mutable_data_id());
  EXPECT_THROW(
      resim::metrics::proto::validate_metrics_data_proto(
          *double_data,
          resim::metrics::proto::build_metrics_data_map(
              job_metrics.metrics_data())),
      AssertException);
  resim::proto::pack(
      double_data_id,
      actor_one_data->mutable_parent_id()->mutable_data_id());

  // Per actor data should validate again after reset
  resim::metrics::proto::validate_metrics_data_proto(
      *double_data,
      resim::metrics::proto::build_metrics_data_map(
          job_metrics.metrics_data()));
}

TEST_F(MetricsProtoTest, IndexedDoubleSummaries) {
  JobMetrics job_metrics;
  UUID job_id = UUID::new_uuid();
  job_metrics.set_metrics_status(MetricStatus::FAILED);
  resim::proto::pack(job_id, job_metrics.mutable_job_id()->mutable_job_id());
  job_metrics.mutable_job_metrics()->set_metrics_status(MetricStatus::FAILED);

  // Make some timestamp data
  auto* timestamp_data = job_metrics.add_metrics_data();
  UUID timestamp_data_id = UUID::new_uuid();
  resim::proto::pack(
      timestamp_data_id,
      timestamp_data->mutable_id()->mutable_data_id());
  timestamp_data->set_data_type(MetricsDataType::TIMESTAMP_ARRAY_DATA_TYPE);
  timestamp_data->set_length(2);
  timestamp_data->set_name("Timestamps");
  timestamp_data->set_is_per_actor(false);
  resim::time::proto::pack(
      time::Timestamp(std::chrono::seconds(0)),
      timestamp_data->mutable_array()->mutable_timestamps()->add_timestamps());
  resim::time::proto::pack(
      time::Timestamp(std::chrono::seconds(1)),
      timestamp_data->mutable_array()->mutable_timestamps()->add_timestamps());
  validate_metrics_data_proto(
      *timestamp_data,
      build_metrics_data_map(job_metrics.metrics_data()));

  // Make some actor ID data
  auto* actor_data = job_metrics.add_metrics_data();
  UUID actor_data_id = UUID::new_uuid();
  resim::proto::pack(
      actor_data_id,
      actor_data->mutable_id()->mutable_data_id());
  actor_data->set_data_type(MetricsDataType::ACTOR_ID_ARRAY_DATA_TYPE);
  actor_data->set_length(2);
  actor_data->set_name("Actors");
  actor_data->set_is_per_actor(false);
  UUID actor_one_id = UUID::new_uuid();
  UUID actor_two_id = UUID::new_uuid();
  resim::proto::pack(
      actor_one_id,
      actor_data->mutable_array()
          ->mutable_actor_ids()
          ->add_actor_ids()
          ->mutable_actor_id());
  resim::proto::pack(
      actor_two_id,
      actor_data->mutable_array()
          ->mutable_actor_ids()
          ->add_actor_ids()
          ->mutable_actor_id());

  // Make some indexed double data
  auto* indexed_double_data = job_metrics.add_metrics_data();
  UUID indexed_double_data_id = UUID::new_uuid();
  resim::proto::pack(
      indexed_double_data_id,
      indexed_double_data->mutable_id()->mutable_data_id());
  indexed_double_data->set_data_type(
      MetricsDataType::INDEXED_DOUBLE_ARRAY_DATA_TYPE);
  indexed_double_data->set_length(2);
  indexed_double_data->set_name("Doubles");
  indexed_double_data->set_is_per_actor(false);
  indexed_double_data->mutable_array()->mutable_indexed_doubles()->add_doubles(
      ONE);
  indexed_double_data->mutable_array()->mutable_indexed_doubles()->add_doubles(
      TWO);

  // Make some indexed status data
  auto* indexed_status_data = job_metrics.add_metrics_data();
  UUID indexed_status_data_id = UUID::new_uuid();
  resim::proto::pack(
      indexed_status_data_id,
      indexed_status_data->mutable_id()->mutable_data_id());
  indexed_status_data->set_data_type(
      MetricsDataType::INDEXED_METRIC_STATUS_ARRAY_DATA_TYPE);
  indexed_status_data->set_length(2);
  indexed_status_data->set_name("Statuses");
  indexed_status_data->set_is_per_actor(false);
  indexed_status_data->mutable_array()
      ->mutable_indexed_statuses()
      ->add_statuses(MetricStatus::FAILED);
  indexed_status_data->mutable_array()
      ->mutable_indexed_statuses()
      ->add_statuses(MetricStatus::PASSED);

  // Make a double summary metric
  auto* metric = job_metrics.mutable_job_metrics()->add_metrics();
  UUID metric_id = UUID::new_uuid();

  resim::proto::pack(metric_id, metric->mutable_id()->mutable_metric_id());
  metric->set_type(MetricType::DOUBLE_SUMMARY);
  metric->set_status(MetricStatus::PASSED);
  resim::proto::pack(job_id, metric->mutable_job_id()->mutable_job_id());

  auto* metric_values =
      metric->mutable_metric_values()->mutable_double_metric_values();

  resim::proto::pack(
      indexed_double_data_id,
      metric_values->mutable_value_data_id()->mutable_data_id());
  resim::proto::pack(
      indexed_status_data_id,
      metric_values->mutable_status_data_id()->mutable_data_id());

  // Index everything by timestamp
  resim::time::proto::pack(
      time::Timestamp(std::chrono::seconds(0)),
      metric_values->mutable_timestamp());
  indexed_double_data->mutable_array()
      ->mutable_indexed_doubles()
      ->set_index_type(MetricsDataType::TIMESTAMP_ARRAY_DATA_TYPE);
  resim::proto::pack(
      timestamp_data_id,
      indexed_double_data->mutable_array()
          ->mutable_indexed_doubles()
          ->mutable_index_id()
          ->mutable_data_id());
  indexed_status_data->mutable_array()
      ->mutable_indexed_statuses()
      ->set_index_type(MetricsDataType::TIMESTAMP_ARRAY_DATA_TYPE);
  resim::proto::pack(
      timestamp_data_id,
      indexed_status_data->mutable_array()
          ->mutable_indexed_statuses()
          ->mutable_index_id()
          ->mutable_data_id());

  // Check validation passes
  resim::metrics::proto::validate_job_metrics_proto(job_metrics);
  metric_values->clear_timestamp();

  // Index both data and statuses by actor ID
  resim::proto::pack(
      actor_one_id,
      metric_values->mutable_actor_id()->mutable_actor_id());
  indexed_double_data->mutable_array()
      ->mutable_indexed_doubles()
      ->set_index_type(MetricsDataType::ACTOR_ID_ARRAY_DATA_TYPE);
  resim::proto::pack(
      actor_data_id,
      indexed_double_data->mutable_array()
          ->mutable_indexed_doubles()
          ->mutable_index_id()
          ->mutable_data_id());
  indexed_status_data->mutable_array()
      ->mutable_indexed_statuses()
      ->set_index_type(MetricsDataType::ACTOR_ID_ARRAY_DATA_TYPE);
  resim::proto::pack(
      actor_data_id,
      indexed_status_data->mutable_array()
          ->mutable_indexed_statuses()
          ->mutable_index_id()
          ->mutable_data_id());

  // Check validation passes
  resim::metrics::proto::validate_job_metrics_proto(job_metrics);
  metric_values->clear_actor_id();

  // Index doubles by ID and statuses by timestamp
  resim::time::proto::pack(
      time::Timestamp(std::chrono::seconds(0)),
      metric_values->mutable_timestamp());
  indexed_double_data->mutable_array()
      ->mutable_indexed_doubles()
      ->set_index_type(MetricsDataType::ACTOR_ID_ARRAY_DATA_TYPE);
  resim::proto::pack(
      actor_data_id,
      indexed_double_data->mutable_array()
          ->mutable_indexed_doubles()
          ->mutable_index_id()
          ->mutable_data_id());
  indexed_status_data->mutable_array()
      ->mutable_indexed_statuses()
      ->set_index_type(MetricsDataType::TIMESTAMP_ARRAY_DATA_TYPE);
  resim::proto::pack(
      timestamp_data_id,
      indexed_status_data->mutable_array()
          ->mutable_indexed_statuses()
          ->mutable_index_id()
          ->mutable_data_id());

  // Validation should fail, as types don't match
  EXPECT_THROW(
      resim::metrics::proto::validate_job_metrics_proto(job_metrics),
      AssertException);
  metric_values->clear_timestamp();

  // Index statuses and data by ID, but provide an array index
  metric_values->set_array_index(1);
  indexed_double_data->mutable_array()
      ->mutable_indexed_doubles()
      ->set_index_type(MetricsDataType::ACTOR_ID_ARRAY_DATA_TYPE);
  resim::proto::pack(
      actor_data_id,
      indexed_double_data->mutable_array()
          ->mutable_indexed_doubles()
          ->mutable_index_id()
          ->mutable_data_id());
  indexed_status_data->mutable_array()
      ->mutable_indexed_statuses()
      ->set_index_type(MetricsDataType::ACTOR_ID_ARRAY_DATA_TYPE);
  resim::proto::pack(
      actor_data_id,
      indexed_status_data->mutable_array()
          ->mutable_indexed_statuses()
          ->mutable_index_id()
          ->mutable_data_id());

  // Validation should fail, as types don't match
  EXPECT_THROW(
      resim::metrics::proto::validate_job_metrics_proto(job_metrics),
      AssertException);
  metric_values->clear_timestamp();
}

}  // namespace resim::metrics::proto
