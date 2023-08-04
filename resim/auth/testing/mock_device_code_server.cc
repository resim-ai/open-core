// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/auth/testing/mock_device_code_server.hh"

#include <google/protobuf/util/json_util.h>

#include <utility>

#include "resim/assert/assert.hh"
#include "resim/utils/http_response.hh"

namespace resim::auth::testing {

namespace {

google::protobuf::util::JsonPrintOptions default_json_options() {
  google::protobuf::util::JsonPrintOptions options;
  options.preserve_proto_field_names = true;
  return options;
}

}  // namespace

MockDeviceCodeServer::MockDeviceCodeServer(
    std::string host,
    std::string device_code,
    int interval,
    std::string token,
    DeviceCodeReceiver &&device_code_receiver,
    PollingReceiver &&polling_receiver)
    : device_code_{std::move(device_code)},
      interval_{interval},
      token_{std::move(token)},
      server_{std::move(host)} {
  server_.add_post_receiver(
      "/oauth/device/code",
      [this, receiver = std::move(device_code_receiver)](
          const std::multimap<std::string, std::string> &,
          const std::string &body,
          const std::smatch &,
          InOut<Response> response) {
        proto::DeviceCodeRequest request;
        const auto parse_status =
            google::protobuf::util::JsonStringToMessage(body, &request);

        REASSERT(parse_status.ok(), "Failed to parse request!");
        receiver(request);

        browser_auth_complete_ = false;

        proto::DeviceCodeResponse response_msg;
        response_msg.set_device_code(device_code_);
        response_msg.set_interval(interval_);
        response_msg.set_expires_in(timeout_s_);

        const auto serialize_status =
            google::protobuf::util::MessageToJsonString(
                response_msg,
                &response->body,
                default_json_options());
        REASSERT(serialize_status.ok(), "Failed to serialize response!");
        response->mime_type = "application/json";
        response->status = HttpResponse::OK;
      });
  server_.add_post_receiver(
      "/oauth/token",
      [this, receiver = std::move(polling_receiver)](
          const std::multimap<std::string, std::string> &,
          const std::string &body,
          const std::smatch &,
          InOut<Response> response) {
        proto::PollingRequest request;
        const auto parse_status =
            google::protobuf::util::JsonStringToMessage(body, &request);

        REASSERT(parse_status.ok(), "Failed to parse request!");

        receiver(request);

        proto::PollingResponse response_msg;

        if (not browser_auth_complete_) {
          response_msg.set_error("authorization_pending");
          response_msg.set_error_description(
              "User has yet to authorize device code.");
          response->status = HttpResponse::FORBIDDEN;
        } else {
          response_msg.set_access_token(token_);
          response->status = HttpResponse::OK;
        }

        const auto serialize_status =
            google::protobuf::util::MessageToJsonString(
                response_msg,
                &response->body,
                default_json_options());
        REASSERT(serialize_status.ok(), "Failed to serialize response!");
        response->mime_type = "application/json";
      });

  server_.listen();
}

std::string MockDeviceCodeServer::host() const { return server_.host(); }

int MockDeviceCodeServer::port() const { return server_.port(); }

void MockDeviceCodeServer::set_browser_auth_complete() {
  browser_auth_complete_ = true;
}

void MockDeviceCodeServer::set_timeout_s(const int timeout_s) {
  timeout_s_ = timeout_s;
}

}  // namespace resim::auth::testing
