#pragma once

#include "resim/auth/proto/auth_messages.pb.h"
#include "resim/testing/mock_server.hh"

namespace resim::auth::testing {

class MockClientCredentialsServer {
 public:
  using ClientCredentialsReceiver = std::function<void(
      const std::string &grant_type,
      const std::string &client_id,
      const std::string &client_secret,
      const std::string &audience)>;

  MockClientCredentialsServer(
      std::string host,
      std::string token,
      ClientCredentialsReceiver &&receiver);

  MockClientCredentialsServer(const MockClientCredentialsServer &) = delete;
  MockClientCredentialsServer(MockClientCredentialsServer &&) = delete;
  MockClientCredentialsServer &operator=(const MockClientCredentialsServer &) =
      delete;
  MockClientCredentialsServer &operator=(MockClientCredentialsServer &&) =
      delete;

  ~MockClientCredentialsServer() = default;

  // Getters for the host and port for the server.
  std::string host() const;
  int port() const;

 private:
  using Response = ::resim::testing::MockServer::Response;

  ::resim::testing::MockServer server_;
  const std::string token_;
};

}  // namespace resim::auth::testing
