#pragma once

#include <filesystem>
#include <string>

#include "resim/auth/auth_client_interface.hh"

namespace resim::auth {

class DeviceCodeClient : public AuthClientInterface {
 public:
  struct Config {
    std::string server;  // The auth server to contact

    // These parameters match the device authorization flow described here:
    // https://auth0.com/docs/get-started/authentication-and-authorization-flow/call-your-api-using-the-device-authorization-flow
    std::string client_id;
    std::string audience;

    // Where to save the token_path
    std::filesystem::path token_path;
  };

  explicit DeviceCodeClient(Config config);

  std::string get_jwt() override;

  void refresh() override;

 private:
  // Load the token from the disk if it exists or fetch and save it if
  // it does not.
  void load_token();

  // Fetch the token from the server, prompting the user to
  // authenticate in the browser.
  void fetch_token();

  // Save the token to the disk in the configured token path.
  void save_token();

  Config config_;
  std::string token_;
};

}  // namespace resim::auth
