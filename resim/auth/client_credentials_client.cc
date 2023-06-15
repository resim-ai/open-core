#include "resim/auth/client_credentials_client.hh"

#include <cpr/cpr.h>
#include <google/protobuf/util/json_util.h>

#include "resim/assert/assert.hh"
#include "resim/auth/proto/auth_messages.pb.h"

namespace resim::auth {

std::string ClientCredentialsClient::get_jwt() {
  // TODO(austin): Check expiration time on the token and refresh automatically.
  if (access_token_.empty()) {
    const cpr::Response response = cpr::Post(
        cpr::Url{config_.server + "/oauth/token"},
        cpr::Payload{
            {"grant_type", "client_credentials"},
            {"client_id", config_.client_id},
            {"client_secret", config_.client_secret},
            {"audience", config_.audience}});
    REASSERT(response.status_code == 200, "Unexpected status code");

    proto::ClientCredentialsResponse auth_response;
    const auto parse_status = google::protobuf::util::JsonStringToMessage(
        response.text,
        &auth_response);
    REASSERT(parse_status.ok(), "Failed to parse response.");

    access_token_ = auth_response.access_token();
  }
  return access_token_;
}

void ClientCredentialsClient::refresh() { access_token_ = ""; }

}  // namespace resim::auth
