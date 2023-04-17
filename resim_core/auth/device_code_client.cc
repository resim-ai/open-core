#include "resim_core/auth/device_code_client.hh"

#include <cpr/cpr.h>
#include <google/protobuf/util/json_util.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <thread>
#include <utility>

#include "resim_core/assert/assert.hh"
#include "resim_core/auth/proto/auth_messages.pb.h"
#include "resim_core/utils/http_response.hh"
#include "resim_core/utils/inout.hh"

namespace resim::auth {

namespace {

// Helper function that wraps up the serialization ugliness so we can just send
// a protobuf message (converted to JSON) to the server and get back a protobuf
// message (converted from a JSON).
template <typename RequestProto, typename ResponseProto>
HttpResponse json_query(
    const std::string &endpoint,
    const RequestProto &request,
    InOut<ResponseProto> response) {
  std::string request_json;
  google::protobuf::util::JsonPrintOptions options;
  options.preserve_proto_field_names = true;
  const auto serialize_status = google::protobuf::util::MessageToJsonString(
      request,
      &request_json,
      options);
  REASSERT(serialize_status.ok(), "Failed to serialize request!");

  const cpr::Response r = cpr::Post(
      cpr::Url{endpoint},
      cpr::Header{{"Content-Type", "application/json"}},
      cpr::Body{request_json});

  const auto parse_status =
      google::protobuf::util::JsonStringToMessage(r.text, &*response);
  REASSERT(parse_status.ok(), "Failed to parse response!");
  return static_cast<HttpResponse>(r.status_code);
}

}  // namespace

DeviceCodeClient::DeviceCodeClient(Config config)
    : config_{std::move(config)} {}

std::string DeviceCodeClient::get_jwt() {
  if (token_.empty()) {
    load_token();
  }
  return token_;
}

void DeviceCodeClient::refresh() {
  // TODO(https://app.asana.com/0/0/1203968428742419/f) Try the refresh token
  // instead of just totally resetting.
  token_.clear();
  std::filesystem::remove_all(config_.token_path);
}

void DeviceCodeClient::save_token() {
  std::filesystem::create_directories(config_.token_path.parent_path());
  std::ofstream token_stream;
  token_stream.open(config_.token_path);
  token_stream << token_;
  token_stream.close();
}

void DeviceCodeClient::load_token() {
  const bool token_exists = std::filesystem::exists(config_.token_path);
  if (not token_exists) {
    fetch_token();
    save_token();
    return;
  }
  std::ostringstream token_ss;
  std::ifstream token_is{config_.token_path};
  token_ss << token_is.rdbuf();
  token_ = token_ss.str();
}

void DeviceCodeClient::fetch_token() {
  proto::DeviceCodeRequest device_code_request;
  device_code_request.set_client_id(config_.client_id);
  device_code_request.set_scope(config_.scope);
  device_code_request.set_audience(config_.audience);

  proto::DeviceCodeResponse device_code_response;
  HttpResponse status = json_query(
      config_.server + "/oauth/device/code",
      device_code_request,
      InOut{device_code_response});

  REASSERT(status == HttpResponse::OK, "Non OK Status detected!");
  std::cout << "Authenticate at the following URL: " +
                   device_code_response.verification_uri_complete()
            << std::endl;

  proto::PollingRequest polling_request;
  polling_request.set_grant_type(
      "urn:ietf:params:oauth:grant-type:device_code");
  polling_request.set_device_code(device_code_response.device_code());
  polling_request.set_client_id(config_.client_id);

  proto::PollingResponse polling_response;

  const auto timeout_time{
      std::chrono::steady_clock::now() +
      std::chrono::seconds(device_code_response.expires_in())};
  while (std::chrono::steady_clock::now() < timeout_time) {
    json_query(
        config_.server + "/oauth/token",
        polling_request,
        InOut{polling_response});

    if (not polling_response.has_error()) {
      REASSERT(
          not polling_response.access_token().empty(),
          "No token received!");
      token_ = polling_response.access_token();
      return;
    }
    std::this_thread::sleep_for(
        std::chrono::seconds(device_code_response.interval()));
  }
  REASSERT(false, "Timed out!");
}

}  // namespace resim::auth
