#pragma once

#include <curl/curl.h>

#include <string>

#include "resim_core/auth/auth_client.hh"
#include "resim_core/utils/status.hh"
#include "resim_core/visualization/proto/view_update.pb.h"
#include "resim_core/visualization/view_client.hh"

namespace resim::visualization {

// A simple client that packs SE3 and sends to server.
class LibcurlClient : public ViewClient {
 public:
  explicit LibcurlClient(std::string base_url);
  Status send_view_update(const ViewUpdate &update) override;

  // Allows overriding the auth client for testing purposes.
  void set_auth_client(std::unique_ptr<auth::AuthClientInterface> auth_client);

 private:
  // Log a url pointing to the session-specific page in the resim app.
  static void log_app_url(std::string url);

  std::unique_ptr<auth::AuthClientInterface> auth_client_;
  CURL *curl_{};
  std::string base_url_;    // base URL for endpoint
  std::string session_id_;  // unique ID per user
  uint64_t update_id_ = 0;  // view update counter
};

}  // namespace resim::visualization
