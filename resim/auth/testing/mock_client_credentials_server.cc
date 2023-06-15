#include "resim/auth/testing/mock_client_credentials_server.hh"

#include <google/protobuf/util/json_util.h>
#include <httplib.h>

#include "resim/utils/http_response.hh"

namespace resim::auth::testing {

MockClientCredentialsServer::MockClientCredentialsServer(
    std::string host,
    std::string token,
    ClientCredentialsReceiver &&receiver)
    : server_{std::move(host)},
      token_{std::move(token)} {
  server_.add_post_receiver(
      "/oauth/token",
      [this, receiver = std::move(receiver)](
          const std::multimap<std::string, std::string> &,
          const std::string &body,
          const std::smatch &,
          InOut<Response> response) {
        httplib::Params params;
        httplib::detail::parse_query_text(body, params);
        const std::string grant_type = params.find("grant_type")->second;
        const std::string client_id = params.find("client_id")->second;
        const std::string client_secret = params.find("client_secret")->second;
        const std::string audience = params.find("audience")->second;
        receiver(grant_type, client_id, client_secret, audience);

        proto::ClientCredentialsResponse response_msg;
        response_msg.set_access_token(token_);
        response_msg.set_token_type("Bearer");
        const int ONE_DAY_IN_SECONDS =
            std::chrono::seconds(std::chrono::days(1)).count();
        response_msg.set_expires_in(ONE_DAY_IN_SECONDS);
        const auto serialize_status =
            google::protobuf::util::MessageToJsonString(
                response_msg,
                &response->body);
        response->mime_type = "application/json";
        response->status = HttpResponse::OK;
      });

  server_.listen();
}

std::string MockClientCredentialsServer::host() const { return server_.host(); }
int MockClientCredentialsServer::port() const { return server_.port(); }
}  // namespace resim::auth::testing
