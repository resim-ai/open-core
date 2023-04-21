#include "resim_core/visualization/testing/mock_server.hh"

#include <cpr/cpr.h>
#include <fmt/core.h>
#include <gtest/gtest.h>

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
  const std::string address = fmt::format("{}:{}", HOST, server.port());

  // ACTION
  const cpr::Response session_response = cpr::Post(
      cpr::Url{address + "/view/sessions"},
      cpr::Bearer{MockServer::valid_token()});

  // VERIFICATION
  EXPECT_EQ(UUID{session_response.text}, test_session_id);
  EXPECT_EQ(
      session_response.status_code,
      static_cast<int>(HttpResponse::CREATED));

  // ACTION
  const std::string endpoint{fmt::format(
      "/view/sessions/{:s}/updates/{:d}",
      test_session_id.to_string(),
      UPDATE_ID)};
  proto::ViewUpdate update_msg;
  pack(test_update, &update_msg);

  const cpr::Response update_response = cpr::Post(
      cpr::Url{address + endpoint},
      cpr::Bearer{MockServer::valid_token()},
      cpr::Body{update_msg.SerializeAsString()},
      cpr::Header{{"Content-Type", "application/octet-stream"}});

  // VERIFICATION
  EXPECT_EQ(
      update_response.status_code,
      static_cast<int>(HttpResponse::CREATED));
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
  const std::string address = fmt::format("{}:{}", HOST, server.port());
  const std::string endpoint{fmt::format(
      "/view/sessions/{:s}/updates/{:d}",
      test_session_id.to_string(),
      UPDATE_ID)};

  proto::ViewUpdate update_msg;
  pack(test_update, &update_msg);

  // ACTION
  const cpr::Response session_response = cpr::Post(
      cpr::Url{address + endpoint},
      cpr::Bearer{MockServer::valid_token()},
      cpr::Body(update_msg.SerializeAsString()));

  // VERIFICATION
  EXPECT_EQ(
      session_response.status_code,
      static_cast<int>(HttpResponse::NOT_FOUND));
  EXPECT_TRUE(checks_have_run);
}

class MockServerHeadersTest
    : public ::testing::TestWithParam<
          std::tuple<std::string, std::pair<cpr::Header, HttpResponse>>> {
 public:
  MockServerHeadersTest()
      : server_{
            HOST,
            TEST_SESSION_ID,
            [](const ViewUpdate &update,
               const UUID &session_id,
               const uint64_t update_id) {
              return ViewSessionUpdateResponse();
            }} {}
  void SetUp() override {
    ASSERT_NE(server_.port(), 0);
    ASSERT_EQ(server_.host(), HOST);
  }

  const MockServer &server() const { return server_; }

  static uint64_t update_id() { return UPDATE_ID; }
  static UUID test_session_id() { return TEST_SESSION_ID; }

 private:
  static const UUID TEST_SESSION_ID;
  static constexpr uint64_t UPDATE_ID = 2U;
  const MockServer server_;
};

const UUID MockServerHeadersTest::TEST_SESSION_ID = UUID::new_uuid();

const std::vector<std::string> endpoints{
    "/view/sessions",
    fmt::format(
        "/view/sessions/{:s}/updates/{:d}",
        MockServerHeadersTest::test_session_id().to_string(),
        MockServerHeadersTest::update_id())};

const std::vector<std::pair<cpr::Header, HttpResponse>> header_cases{
    {{{"Authorization", "Bearer " + MockServer::valid_token()}},
     HttpResponse::CREATED},
    {{{"Authorization", "Bearer gibberish"}}, HttpResponse::UNAUTHORIZED},
    {{{"Authorization", "Bearer"}}, HttpResponse::UNAUTHORIZED},
    {{{"Authorization", "Bear"}}, HttpResponse::UNAUTHORIZED},
    {{{"Authorization", "Bear misspelled"}}, HttpResponse::UNAUTHORIZED},
    {{{"Authorization", "Bearer " + MockServer::unauthorized_token()}},
     HttpResponse::UNAUTHORIZED},
    {{{"Authorization", "Bearer " + MockServer::forbidden_token()}},
     HttpResponse::FORBIDDEN},
    {{}, HttpResponse::UNAUTHORIZED},
};

INSTANTIATE_TEST_SUITE_P(
    EndpointsAndTokens,
    MockServerHeadersTest,
    ::testing::Combine(
        ::testing::ValuesIn(endpoints),
        ::testing::ValuesIn(header_cases)));

TEST_P(MockServerHeadersTest, TestChecksAuthorization) {
  // SETUP
  const std::string address = fmt::format("{}:{}", HOST, server().port());
  const auto [endpoint, header_case] = GetParam();
  const auto [headers, return_code] = header_case;

  // ACTION
  const cpr::Response response =
      cpr::Post(cpr::Url{address + endpoint}, headers);
  EXPECT_EQ(response.status_code, static_cast<int>(return_code));
}
}  // namespace resim::visualization::testing
