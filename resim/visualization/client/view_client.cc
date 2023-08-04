// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/visualization/client/view_client.hh"

#include <cpr/cpr.h>
#include <fmt/core.h>
#include <glog/logging.h>
#include <google/protobuf/util/json_util.h>
#include <sys/types.h>

#include <algorithm>
#include <cstdint>
#include <string>

#include "resim/assert/assert.hh"
#include "resim/auth/device_code_client.hh"
#include "resim/utils/http_response.hh"
#include "resim/utils/status.hh"
#include "resim/visualization/client/proto/view_client.pb.h"
#include "resim/visualization/proto/view_update_to_proto.hh"

namespace resim::visualization {

ViewClient::ViewClient(std::string base_url) : base_url_{std::move(base_url)} {
  // TODO(https://app.asana.com/0/1204498029712344/1204809004921025/f) Don't
  // hard code these.
  auth::DeviceCodeClient::Config auth_client_config = {
      .server = "https://resim.us.auth0.com",
      .client_id = "RNWJ3idL2UadBBzDqAaq11G9JIgMh7iy",
      .audience = "https://api.resim.ai",
      .token_path = determine_token_root(getenv("HOME")),
  };
  auth_client_ = std::make_unique<auth::DeviceCodeClient>(auth_client_config);
}

Status ViewClient::send_view_update(const ViewUpdate &update) {
  REASSERT(!base_url_.empty(), "Endpoint URL must not be empty!");

  if (session_id_.empty()) {
    cpr::Response response = cpr::Post(
        cpr::Url{base_url_ + "/view/sessions"},
        cpr::Bearer{auth_client_->get_jwt()});

    if (response.status_code == static_cast<int>(HttpResponse::UNAUTHORIZED)) {
      auth_client_->refresh();
      response = cpr::Post(
          cpr::Url{base_url_ + "/view/sessions"},
          cpr::Bearer{auth_client_->get_jwt()});
    }

    if (response.status_code != static_cast<int>(HttpResponse::CREATED)) {
      LOG(ERROR) << "Could not create session. HTTP Status: "
                 << response.status_code;
      LOG(ERROR) << response.text;
      return MAKE_STATUS("Could not create session.");
    }

    session_id_ = response.text;
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

  cpr::Response response = cpr::Post(
      url,
      cpr::Header{
          {"Content-Type", "application/octet-stream"},
      },
      cpr::Bearer(auth_client_->get_jwt()),
      cpr::Body{serialized_output});

  if (response.status_code == static_cast<int>(HttpResponse::UNAUTHORIZED)) {
    auth_client_->refresh();
    response = cpr::Post(
        url,
        cpr::Header{
            {"Content-Type", "application/octet-stream"},
        },
        cpr::Bearer(auth_client_->get_jwt()),
        cpr::Body{serialized_output});
  }

  if (response.status_code != static_cast<int>(HttpResponse::CREATED)) {
    LOG(ERROR) << "Could not send view update. HTTP Status code: "
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
