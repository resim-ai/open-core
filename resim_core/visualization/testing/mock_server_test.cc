#include "resim_core/visualization/testing/mock_server.hh"

#include <fmt/core.h>
#include <gtest/gtest.h>
#include <httplib.h>

#include <cstdint>

#include "resim_core/transforms/se3.hh"
#include "resim_core/utils/http_response.hh"
#include "resim_core/utils/uuid.hh"
#include "resim_core/visualization/proto/view_update.pb.h"
#include "resim_core/visualization/proto/view_update_to_proto.hh"
#include "resim_core/visualization/testing/test_helpers.hh"
#include "resim_core/visualization/view_primitive.hh"
#include "resim_core/visualization/view_update.hh"

namespace resim::visualization::testing {

using ::resim::visualization::client::proto::ViewSessionUpdateResponse;
using transforms::SE3;
using TangentVector = SE3::TangentVector;

namespace {

constexpr int CREATED = 201;
constexpr auto HOST = "localhost";

const TangentVector test_tangent{
    (TangentVector() << 1., 2., 3., 4., 5., 6.).finished()};

const SE3 test_se3{SE3::exp(test_tangent)};

const ViewPrimitive test_primitive{
    .id = UUID::new_uuid(),
    .payload = test_se3,
};

const ViewUpdate test_update{
    .primitives = {test_primitive, test_primitive},
};

// A simple receiver that checks that the update, session_id, and update_id that
// the server receives match what they should be.
struct TestReceiver {
  const ViewUpdate &expected_update;
  const UUID &expected_session_id;
  const uint64_t expected_update_id;
  bool &checks_have_run;

  ViewSessionUpdateResponse operator()(
      const ViewUpdate &update,
      const UUID &session_id,
      const uint64_t update_id) {
    // Verify that we get back from the server what we expect to:
    EXPECT_EQ(update.primitives.size(), expected_update.primitives.size());
    for (std::size_t ii = 0U; ii < update.primitives.size(); ++ii) {
      EXPECT_TRUE(primitives_equal(
          update.primitives.at(ii),
          test_update.primitives.at(ii)));
    }
    EXPECT_EQ(expected_session_id, session_id);
    EXPECT_EQ(expected_update_id, update_id);
    checks_have_run = true;

    return ViewSessionUpdateResponse{};
  }
};

}  // namespace

TEST(MockServerTest, TestConstruction) {
  // SETUP
  const UUID test_session_id{UUID::new_uuid()};
  constexpr uint64_t UPDATE_ID = 2U;

  bool checks_have_run = false;
  MockServer server{
      HOST,
      test_session_id,
      TestReceiver{test_update, test_session_id, UPDATE_ID, checks_have_run}};

  ASSERT_NE(server.port(), 0);
  ASSERT_EQ(server.host(), HOST);

  httplib::Client cli{HOST, server.port()};
  httplib::Headers headers{{"Authorization", "Bearer test"}};

  // ACTION
  httplib::Result res{cli.Post("/view/sessions", headers, "", "text/plain")};

  // VERIFICATION
  EXPECT_EQ(UUID{res.value().body}, test_session_id);
  EXPECT_EQ(res->status, CREATED);

  // ACTION
  const std::string endpoint{fmt::format(
      "/view/sessions/{:s}/updates/{:d}",
      test_session_id.to_string(),
      UPDATE_ID)};
  proto::ViewUpdate update_msg;
  pack(test_update, &update_msg);
  res = cli.Post(
      endpoint,
      headers,
      update_msg.SerializeAsString(),
      "application/octet-stream");

  // VERIFICATION
  EXPECT_EQ(res->status, CREATED);
  EXPECT_TRUE(checks_have_run);
}

// Test that we can return another code from the ViewUpdate post if we set it.
TEST(MockServerTest, TestReturnsRequestedCode) {
  // SETUP
  const UUID test_session_id{UUID::new_uuid()};
  constexpr uint64_t UPDATE_ID = 2U;

  bool checks_have_run = false;
  MockServer server{
      HOST,
      test_session_id,
      TestReceiver{test_update, test_session_id, UPDATE_ID, checks_have_run},
      HttpResponse::NOT_FOUND};

  ASSERT_NE(server.port(), 0);
  ASSERT_EQ(server.host(), HOST);

  // Get the session
  httplib::Client cli{HOST, server.port()};

  const std::string endpoint{fmt::format(
      "/view/sessions/{:s}/updates/{:d}",
      test_session_id.to_string(),
      UPDATE_ID)};
  proto::ViewUpdate update_msg;
  pack(test_update, &update_msg);

  httplib::Headers headers{{"Authorization", "Bearer test"}};

  // ACTION
  const httplib::Result res = cli.Post(
      endpoint,
      headers,
      update_msg.SerializeAsString(),
      "application/octet-stream");

  // VERIFICATION
  EXPECT_EQ(res->status, static_cast<int>(HttpResponse::NOT_FOUND));
  EXPECT_TRUE(checks_have_run);
}

TEST(MockServerTest, TestChecksAuthorization) {
  // SETUP
  const UUID test_session_id{UUID::new_uuid()};
  constexpr uint64_t UPDATE_ID = 2U;
  MockServer server{
      HOST,
      test_session_id,
      [](const ViewUpdate &, UUID, uint64_t) {
        return ViewSessionUpdateResponse();
      }};

  ASSERT_NE(server.port(), 0);
  ASSERT_EQ(server.host(), HOST);

  httplib::Client cli{HOST, server.port()};

  const std::vector<std::string> endpoints{
      "/view/sessions",
      fmt::format(
          "/view/sessions/{:s}/updates/{:d}",
          test_session_id.to_string(),
          UPDATE_ID)};
  const std::vector<std::pair<httplib::Headers, HttpResponse>> cases{
      {{{"Authorization", "Bearer foo"}}, HttpResponse::CREATED},
      {{{"Authorization", "Bearer"}}, HttpResponse::FORBIDDEN},
      {{{"Authorization", "Bear"}}, HttpResponse::FORBIDDEN},
      {{}, HttpResponse::UNAUTHORIZED},
  };

  for (const auto &endpoint : endpoints) {
    for (const auto &c : cases) {
      const httplib::Headers &headers = c.first;
      const HttpResponse &return_code = c.second;

      // ACTION
      httplib::Result res = cli.Post(endpoint, headers, "", "text/plain");

      // VERIFICATION
      EXPECT_EQ(res->status, static_cast<int>(return_code));
    }
  }
}

}  // namespace resim::visualization::testing
