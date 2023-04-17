#include "resim_core/visualization/client/view_client.hh"

#include <cpr/cpr.h>
#include <fmt/core.h>
#include <glog/logging.h>
#include <google/protobuf/util/json_util.h>
#include <sys/types.h>

#include <algorithm>
#include <cstdint>
#include <string>

#include "resim_core/assert/assert.hh"
#include "resim_core/auth/device_code_client.hh"
#include "resim_core/utils/http_response.hh"
#include "resim_core/utils/status.hh"
#include "resim_core/visualization/client/proto/view_client.pb.h"
#include "resim_core/visualization/proto/view_update_to_proto.hh"

namespace resim::visualization {

ViewClient::ViewClient(std::string base_url) : base_url_{std::move(base_url)} {
  auth::DeviceCodeClient::Config auth_client_config = {
      .server = "https://dev-jbs5inutkvkmnq7c.us.auth0.com",
      .client_id = "k0ib8T3aTchwCOZvh40xFw0X4aZeduRS",
      .scope = "view:all",
      .audience = "https://api.resim.ai",
      .token_path = determine_token_root(getenv("HOME")),
  };
  auth_client_ = std::make_unique<auth::DeviceCodeClient>(auth_client_config);
}

Status ViewClient::send_view_update(const ViewUpdate &update) {
  REASSERT(!base_url_.empty(), "Endpoint URL must not be empty!");

  if (session_id_.empty()) {
    const cpr::Response response = cpr::Post(
        cpr::Url{base_url_ + "/view/sessions"},
        cpr::Bearer{auth_client_->get_jwt()});

    // If the session_id_ is still empty, then we cannot do anything.
    session_id_ = response.text;
    if (session_id_.empty()) {
      return MAKE_STATUS("Empty session ID.");
    }

    // Parse the string to remove double quotes and newline character.
    session_id_.erase(
        std::remove(session_id_.begin(), session_id_.end(), '\"'),
        session_id_.end());
    session_id_.erase(
        std::remove(session_id_.begin(), session_id_.end(), '\n'),
        session_id_.end());
  }

  // Pack ViewUpdate and serialize to string.
  proto::ViewUpdate out;
  pack(update, &out);
  std::string serialized_output{out.SerializeAsString()};

  cpr::Url url{fmt::format(
      "{}/view/sessions/{}/updates/{}",
      base_url_,
      session_id_,
      std::to_string(update_id_))};

  // Increment the update_id_.
  update_id_++;

  const cpr::Response response = cpr::Post(
      url,
      cpr::Header{
          {"Content-Type", "application/octet-stream"},
      },
      cpr::Bearer(auth_client_->get_jwt()),
      cpr::Body{serialized_output});

  if (response.status_code != static_cast<int>(HttpResponse::CREATED)) {
    LOG(ERROR) << "Could not send view update.  HTTP Response code: "
               << response.status_code;
    LOG(ERROR) << response.text;
    return MAKE_STATUS("Could not send view update.");
  }

  // Parse the response.
  client::proto::ViewSessionUpdateResponse response_proto;
  google::protobuf::util::JsonStringToMessage(response.text, &response_proto);

  // Return the ReSim App page URL to the user.
  log_app_url(response_proto.view());

  return OKAY_STATUS;
}

void ViewClient::set_auth_client(
    std::unique_ptr<auth::AuthClientInterface> auth_client) {
  auth_client_ = std::move(auth_client);
}

std::filesystem::path ViewClient::determine_token_root(const char *home) {
  std::filesystem::path root =
      home == nullptr ? std::filesystem::path(getenv("TEST_TMPDIR"))
                      : std::filesystem::path(home);
  return root / ".resim" / "view_token";
}

void ViewClient::log_app_url(const std::string &url) {
  const auto url_message =
      fmt::format("View your data visualization in the ReSim App: {}", url);
  LOG(INFO) << url_message;
}
}  // namespace resim::visualization
