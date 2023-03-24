#include "resim_core/auth/device_code_client.hh"

#include <gtest/gtest.h>

#include <utility>

#include "resim_core/assert/assert.hh"
#include "resim_core/auth/testing/mock_device_code_server.hh"
#include "resim_core/testing/test_directory.hh"

namespace resim::auth {

class AuthClientTest : public ::testing::Test {
 protected:
  static constexpr auto TOKEN = "Some Token";
  static constexpr auto DEVICE_CODE = "Some Device Code";
  static constexpr int EXPECTED_POLL_COUNT = 3;

  static constexpr auto HOST = "localhost";

  // We can get away with a zero second interval since we're running
  // the server locally.
  static constexpr int INTERVAL_S = 0;

  AuthClientTest()
      : config_{
            .client_id = "Some Client Id",
            .scope = "view:all",
            .audience = "https://api.resim.ai",
            .token_path = test_directory_.test_file_path("jwt"),
        } {
    server_ = std::make_unique<testing::MockDeviceCodeServer>(
        HOST,
        DEVICE_CODE,
        INTERVAL_S,
        TOKEN,
        [this](const proto::DeviceCodeRequest &request) {
          handle_device_code_request(request);
        },
        [this](const proto::PollingRequest &request) {
          handle_polling_request(request);
        });
    config_.server = server_->host() + ":" + std::to_string(server_->port());
  }

  void handle_device_code_request(const proto::DeviceCodeRequest &request) {
    EXPECT_EQ(request.client_id(), config_.client_id);
    EXPECT_EQ(request.scope(), config_.scope);
    EXPECT_EQ(request.audience(), config_.audience);
    ++device_code_count_;
  }

  void handle_polling_request(const proto::PollingRequest &request) {
    EXPECT_EQ(
        request.grant_type(),
        "urn:ietf:params:oauth:grant-type:device_code");
    EXPECT_EQ(request.device_code(), DEVICE_CODE);
    EXPECT_EQ(request.client_id(), config_.client_id);

    ++polling_count_;
    if (polling_count_ >= EXPECTED_POLL_COUNT) {
      server_->set_browser_auth_complete();
    }
  }

  void reset_counts() {
    device_code_count_ = 0;
    polling_count_ = 0;
  }

  int device_code_count() const { return device_code_count_; }
  int polling_count() const { return polling_count_; }
  const DeviceCodeClient::Config &config() { return config_; }

  const std::unique_ptr<testing::MockDeviceCodeServer> &mutable_server() {
    return server_;
  }

 private:
  resim::testing::TestDirectoryRAII test_directory_;
  int device_code_count_ = 0;
  int polling_count_ = 0;

  DeviceCodeClient::Config config_;
  std::unique_ptr<testing::MockDeviceCodeServer> server_;
};

TEST_F(AuthClientTest, TestFetch) {
  // Test that we can fetch a token from the mock server
  DeviceCodeClient client{config()};
  EXPECT_EQ(client.get_jwt(), TOKEN);
  EXPECT_EQ(device_code_count(), 1);
  EXPECT_EQ(polling_count(), EXPECTED_POLL_COUNT);
}

TEST_F(AuthClientTest, TestCaching) {
  // Test that this client caches the token so we don't have to query the
  // server.
  DeviceCodeClient client{config()};
  EXPECT_EQ(client.get_jwt(), TOKEN);

  reset_counts();
  EXPECT_EQ(client.get_jwt(), TOKEN);
  EXPECT_EQ(device_code_count(), 0);
  EXPECT_EQ(polling_count(), 0);
}

TEST_F(AuthClientTest, TestLoadFromDisk) {
  // Test that we can load a token from disk without querying the server
  {
    // Pre-fetch in another instance so the token is stored on the disk
    EXPECT_EQ(DeviceCodeClient{config()}.get_jwt(), TOKEN);
  }
  DeviceCodeClient client{config()};
  reset_counts();
  EXPECT_EQ(client.get_jwt(), TOKEN);
  EXPECT_EQ(device_code_count(), 0);
  EXPECT_EQ(polling_count(), 0);
}

TEST_F(AuthClientTest, TestRefresh) {
  DeviceCodeClient client{config()};
  EXPECT_EQ(client.get_jwt(), TOKEN);

  // Test that refreshing resets things
  reset_counts();
  client.refresh();
  EXPECT_EQ(client.get_jwt(), TOKEN);
  EXPECT_EQ(device_code_count(), 1);
  EXPECT_EQ(polling_count(), EXPECTED_POLL_COUNT);
}

using AuthClientDeathTest = AuthClientTest;

TEST_F(AuthClientDeathTest, TestTimeout) {
  // Test the timeout
  DeviceCodeClient client{config()};
  client.refresh();
  mutable_server()->set_timeout_s(0);
  EXPECT_THROW(client.get_jwt(), AssertException);
}

}  // namespace resim::auth
