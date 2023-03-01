
#include "resim_core/visualization/testing/mock_server.hh"

#include <glog/logging.h>
#include <google/protobuf/util/json_util.h>
#include <httplib.h>

#include <string>
#include <utility>

#include "resim_core/assert/assert.hh"

namespace resim::visualization::testing {

struct ServerHandle {
  httplib::Server server;
};

MockServer::MockServer(
    std::string host,
    const UUID &session_id,
    Receiver &&receiver,
    const ResponseCode view_update_response_code)
    : host_{std::move(host)},
      receiver_{std::move(receiver)},
      view_update_response_code_{view_update_response_code},
      handle_{std::make_unique<ServerHandle>()},
      session_id_{session_id} {
  // Handler for session posts
  handle_->server.Post(
      "/view/sessions",
      [this](
          const httplib::Request &,
          httplib::Response &res,
          const httplib::ContentReader &) {
        LOG(INFO) << "Session ID Requested...";
        res.set_content(session_id_.to_string(), "text/plain");
        res.status = static_cast<int>(ResponseCode::CREATED);
      });

  // Handler for ViewUpdate posts
  handle_->server.Post(
      "/view/sessions/(.*)/updates/(.*)",
      [this](
          const httplib::Request &req,
          httplib::Response &res,
          const httplib::ContentReader &reader) {
        LOG(INFO) << "ViewUpdate Received...";

        // Get the body
        std::string body;
        reader([&](const char *data, const size_t data_length) {
          body.append(data, data_length);
          return true;
        });

        // Deserialize it
        proto::ViewUpdate update_msg;
        const bool success = update_msg.ParseFromString(body);
        REASSERT(success, "Failed to parse ViewUpdate!");
        LOG(INFO) << "Parsed ViewUpdate Successfully...";

        constexpr std::size_t NUM_MATCHES = 3U;
        constexpr auto ERR_MSG{"Wrong number of regex matches!"};
        REASSERT(req.matches.size() == NUM_MATCHES, ERR_MSG);

        const UUID session_id{req.matches[1]};
        const uint64_t update_id{std::stoul(req.matches[2])};

        // Call the receiver
        client::proto::ViewSessionUpdateResponse response =
            receiver_(unpack(update_msg), session_id, update_id);
        res.status = static_cast<int>(view_update_response_code_);

        std::string response_string;
        google::protobuf::util::MessageToJsonString(response, &response_string);
        res.set_content(response_string, "application/json");
      });

  // Stop handler so we can stop the server from another thread
  handle_->server.Get(
      "/stop",
      [this](const httplib::Request &, httplib::Response &res) {
        LOG(INFO) << "Stopping mock server...";
        handle_->server.stop();
      });

  // Get handler so we can wait until the server is ready before
  // returning from the constructor.
  handle_->server.Get(
      "/ready",
      [](const httplib::Request &, httplib::Response &res) {
        LOG(INFO) << "Mock server ready...";
        res.status = static_cast<int>(ResponseCode::CREATED);
      });

  port_ = handle_->server.bind_to_any_port(host_);
  server_thread_ = std::make_unique<std::thread>(
      [this]() { handle_->server.listen_after_bind(); });

  // Make sure the server is ready
  {
    httplib::Client cli{host_, port_};
    const httplib::Result res{cli.Get("/ready")};
    const bool got_created =
        res->status == static_cast<int>(ResponseCode::CREATED);
    REASSERT(got_created, "Failed to setup serve");
  }
}

MockServer::~MockServer() {
  // Kill the server
  httplib::Client cli{host_, port_};
  cli.Get("/stop");
  server_thread_->join();
}

std::string MockServer::host() const { return host_; }

int MockServer::port() const { return port_; }

}  // namespace resim::visualization::testing
