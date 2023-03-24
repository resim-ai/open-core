#include "resim_core/auth/client_credentials_client.hh"

#include <gtest/gtest.h>

#include <memory>

#include "resim_core/auth/testing/mock_client_credentials_server.hh"

namespace resim::auth::testing {

class ClientCredentialsClientTest : public ::testing::Test {
 public:
  static constexpr auto TOKEN = "test JWT token";
  ClientCredentialsClientTest()
      : config_{
            .client_id = "Test Client ID",
            .client_secret = "Test Client Secret",
            .audience = "Test Audience",
        } {
    server_ = std::make_unique<testing::MockClientCredentialsServer>(
        "localhost",
        TOKEN,
        [this](
            const std::string &grant_type,
            const std::string &client_id,
            const std::string &client_secret,
            const std::string &audience) {
          EXPECT_EQ(grant_type, "client_credentials");
          EXPECT_EQ(client_id, config_.client_id);
          EXPECT_EQ(client_secret, config_.client_secret);
          EXPECT_EQ(audience, config_.audience);
        });

    config_.server = server_->host() + ":" + std::to_string(server_->port());
  }

  const ClientCredentialsClient::Config &config() { return config_; }

 private:
  ClientCredentialsClient::Config config_;
  std::unique_ptr<MockClientCredentialsServer> server_;
};

TEST_F(ClientCredentialsClientTest, TestFetch) {
  ClientCredentialsClient client{config()};
  EXPECT_EQ(client.get_jwt(), TOKEN);
}

TEST_F(ClientCredentialsClientTest, TestRefresh) {
  ClientCredentialsClient client{config()};
  EXPECT_EQ(client.get_jwt(), TOKEN);
  client.refresh();
  EXPECT_EQ(client.get_jwt(), TOKEN);
}

}  // namespace resim::auth::testing
