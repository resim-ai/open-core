#include "resim_core/auth/client_credentials_client.hh"

#include <google/protobuf/util/json_util.h>
#include <httplib.h>

#include "resim_core/assert/assert.hh"
#include "resim_core/auth/proto/auth_messages.pb.h"

namespace resim::auth {

std::string ClientCredentialsClient::get_jwt() {
  // TODO(austin): Check expiration time on the token and refresh automatically.
  if (access_token_.empty()) {
    httplib::Client client{config_.server};
    httplib::Params params;
    params.emplace("grant_type", "client_credentials");
    params.emplace("client_id", config_.client_id);
    params.emplace("client_secret", config_.client_secret);
    params.emplace("audience", config_.audience);
    const httplib::Result result = client.Post("/oauth/token", params);
    REASSERT(result->status == 200, "Unexpected status code");

    proto::ClientCredentialsResponse auth_response;
    const auto parse_status = google::protobuf::util::JsonStringToMessage(
        result->body,
        &auth_response);
    REASSERT(parse_status.ok(), "Failed to parse response.");

    access_token_ = auth_response.access_token();
  }
  return access_token_;
}

void ClientCredentialsClient::refresh() { access_token_ = ""; }

}  // namespace resim::auth
