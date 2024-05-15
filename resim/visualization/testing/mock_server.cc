// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/visualization/testing/mock_server.hh"

#include <glog/logging.h>
#include <google/protobuf/util/json_util.h>
#include <httplib.h>

#include <string>
#include <utility>

#include "resim/assert/assert.hh"
#include "resim/utils/http_response.hh"

namespace resim::visualization::testing {

const std::string MockServer::VALID_TOKEN_ = "valid token";
const std::string MockServer::UNAUTHORIZED_TOKEN_ = "unauthorized token";
const std::string MockServer::FORBIDDEN_TOKEN_ = "forbidden token";

struct ServerHandle {
  httplib::Server server;
};

namespace {

bool validate_headers(
    const std::multimap<std::string, std::string> &headers,
    InOut<::resim::testing::MockServer::Response> response) {
  if (not headers.contains("Authorization")) {
    response->status = HttpResponse::UNAUTHORIZED;
    return false;
  }
  bool found = false;
  const auto &range = headers.equal_range("Authorization");
  std::string token;
  for (auto it = range.first; it != range.second; ++it) {
    const size_t loc = it->second.find("Bearer ");
    if (loc == 0) {
      found = true;
      token = it->second.substr(std::string("Bearer ").size());
      break;
    }
  }
  if (!found || token == MockServer::unauthorized_token()) {
    response->status = HttpResponse::UNAUTHORIZED;
    return false;
  }
  if (token == MockServer::forbidden_token()) {
    response->status = HttpResponse::FORBIDDEN;
    return false;
  }
  if (token == MockServer::valid_token()) {
    return true;
  }
  response->status = HttpResponse::UNAUTHORIZED;
  return false;
}

}  // namespace

MockServer::MockServer(
    std::string host,
    const UUID &session_id,
    Receiver &&receiver,
    const HttpResponse view_update_response_code)
    : receiver_{std::move(receiver)},
      view_update_response_code_{view_update_response_code},
      session_id_{session_id},
      server_{std::move(host)} {
  // Handler for session posts
  server_.add_post_receiver(
      "/view/sessions",
      [this](
          const std::multimap<std::string, std::string> &headers,
          const std::string &body,
          const std::smatch &,
          InOut<::resim::testing::MockServer::Response> response) {
        if (!validate_headers(headers, response)) {
          return;
        }

        LOG(INFO) << "Session ID Requested...";
        response->body = session_id_.to_string();
        response->mime_type = "text/plain";
        response->status = HttpResponse::CREATED;
      });

  // Handler for ViewUpdate post
  server_.add_post_receiver(
      "/view/sessions/(.*)/updates/(.*)",
      [this](
          const std::multimap<std::string, std::string> &headers,
          const std::string &body,
          const std::smatch &matches,
          InOut<::resim::testing::MockServer::Response> response) {
        if (!validate_headers(headers, response)) {
          return;
        }

        LOG(INFO) << "ViewUpdate Received...";

        // Deserialize the body
        proto::ViewUpdate update_msg;
        const bool success = update_msg.ParseFromString(body);
        REASSERT(success, "Failed to parse ViewUpdate!");
        LOG(INFO) << "Parsed ViewUpdate Successfully...";

        constexpr std::size_t NUM_MATCHES = 3U;
        constexpr auto ERR_MSG{"Wrong number of regex matches!"};
        REASSERT(matches.size() == NUM_MATCHES, ERR_MSG);

        const UUID session_id{matches[1]};
        const uint64_t update_id{std::stoul(matches[2])};

        // Call the receiver
        receiver_(unpack(update_msg), session_id, update_id);
        response->status = view_update_response_code_;

        client::proto::ViewSessionUpdateResponse response_msg =
            receiver_(unpack(update_msg), session_id, update_id);
        std::string response_string;
        REASSERT(google::protobuf::util::MessageToJsonString(
                     response_msg,
                     &response_string)
                     .ok());
        response->body = response_string;
        response->mime_type = "application/json";
      });

  server_.listen();
}

std::string MockServer::host() const { return server_.host(); }

int MockServer::port() const { return server_.port(); }

}  // namespace resim::visualization::testing
