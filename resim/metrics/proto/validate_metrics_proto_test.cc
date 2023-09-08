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
#include "tools/cpp/runfiles/runfiles.h"

namespace resim::metrics::proto {

class MetricsProtoTest : public ::testing::Test {
 public:
  static std::unique_ptr<bazel::tools::cpp::runfiles::Runfiles> runfiles;

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

std::unique_ptr<bazel::tools::cpp::runfiles::Runfiles>
    MetricsProtoTest::runfiles;

JobMetrics read_metrics_json(std::string metrics_path) {
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
  resim::metrics::proto::validate_job_metrics_proto(metrics);
}

}  // namespace resim::metrics::proto
