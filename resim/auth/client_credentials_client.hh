#pragma once

#include <string>

#include "resim/auth/auth_client_interface.hh"

namespace resim::auth {

// ClientCredentialsClient is an AuthClient that implements the client
// credentials flow. That flow is described here
// https://auth0.com/docs/get-started/authentication-and-authorization-flow/client-credentials-flow
// In short, the client exchanges the client_id and the client_secret with auth0
// and receives a JWT for the specified audience.
class ClientCredentialsClient : public AuthClientInterface {
 public:
  struct Config {
    // the URL of the authentication server (no endpoint specified)
    std::string server;
    std::string client_id;      // specified by the auth server
    std::string client_secret;  // specified by the auth server
    // the audience to request permissions for (e.g. "view:all")
    std::string audience;
  };

  explicit ClientCredentialsClient(Config config)
      : config_(std::move(config)) {}

  std::string get_jwt() override;

  void refresh() override;

 private:
  Config config_;
  std::string access_token_;
};

}  // namespace resim::auth
