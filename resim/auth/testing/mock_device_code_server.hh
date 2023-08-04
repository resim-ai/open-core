// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include "resim/auth/proto/auth_messages.pb.h"
#include "resim/testing/mock_server.hh"

namespace resim::auth::testing {

// Run a mock server for testing authentication. This mock is meant to support
// testing the auth0 device authorization flow in unit tests:
// https://auth0.com/docs/get-started/authentication-and-authorization-flow/call-your-api-using-the-device-authorization-flow
class MockDeviceCodeServer {
 public:
  // Receiver called for requests on /oauth/device/code
  using DeviceCodeReceiver =
      std::function<void(const proto::DeviceCodeRequest &)>;

  // Receiver called for requests on /oauth/token
  using PollingReceiver = std::function<void(const proto::PollingRequest &)>;

  // Constructor
  // @param[in] host - Where to host the server.
  // @param[in] device_code - What device code to expect from the client.
  // @param[in] interval - What interval to send to the client.
  // @param[in] token - The token to send to the client.
  // @param[in] device_code_receiver - The receiver to run for
  // /oauth/device/code
  // @param[in] polling_receiver - The receiver to run for /oauth/token
  MockDeviceCodeServer(
      std::string host,
      std::string device_code,
      int interval,
      std::string token,
      DeviceCodeReceiver &&device_code_receiver,
      PollingReceiver &&polling_receiver);

  MockDeviceCodeServer(const MockDeviceCodeServer &) = delete;
  MockDeviceCodeServer(MockDeviceCodeServer &&) = delete;
  MockDeviceCodeServer &operator=(const MockDeviceCodeServer &) = delete;
  MockDeviceCodeServer &operator=(MockDeviceCodeServer &&) = delete;

  ~MockDeviceCodeServer() = default;

  // Getters for the host and port for the server.
  std::string host() const;
  int port() const;

  // Helper to simulate the client completing authentication through the
  // browser.
  void set_browser_auth_complete();

  // Set the timeout (expires_in) to send to the client when responding on
  // /oauth/device/code
  void set_timeout_s(int timeout_s);

 private:
  using Response = ::resim::testing::MockServer::Response;

  std::string device_code_;
  int interval_ = -1;
  std::string token_;

  bool browser_auth_complete_ = false;
  static constexpr int DEFAULT_TIMEOUT_S = 600;
  int timeout_s_ = DEFAULT_TIMEOUT_S;

  ::resim::testing::MockServer server_;
};
}  // namespace resim::auth::testing
