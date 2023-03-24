
#include "resim_core/visualization/testing/mock_server.hh"

#include <glog/logging.h>
#include <google/protobuf/util/json_util.h>
#include <httplib.h>

#include <string>
#include <utility>

#include "resim_core/assert/assert.hh"
#include "resim_core/utils/http_response.hh"

namespace resim::visualization::testing {

struct ServerHandle {
  httplib::Server server;
};

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
        if (not headers.contains("Authorization")) {
          response->status = HttpResponse::UNAUTHORIZED;
          return;
        }
        bool found = false;
        const auto &range = headers.equal_range("Authorization");
        for (auto it = range.first; it != range.second; ++it) {
          if (it->second.find("Bearer ") == 0) {
            found = true;
            break;
          }
        }
        if (!found) {
          response->status = HttpResponse::FORBIDDEN;
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
        if (not headers.contains("Authorization")) {
          response->status = HttpResponse::UNAUTHORIZED;
          return;
        }
        bool found = false;
        const auto &range = headers.equal_range("Authorization");
        for (auto it = range.first; it != range.second; ++it) {
          if (it->second.find("Bearer ") == 0) {
            found = true;
            break;
          }
        }
        if (!found) {
          response->status = HttpResponse::FORBIDDEN;
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
        google::protobuf::util::MessageToJsonString(
            response_msg,
            &response_string);
        response->body = response_string;
        response->mime_type = "application/json";
      });

  server_.listen();
}

std::string MockServer::host() const { return server_.host(); }

int MockServer::port() const { return server_.port(); }

}  // namespace resim::visualization::testing
