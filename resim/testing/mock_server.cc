// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/testing/mock_server.hh"

#include <glog/logging.h>
#include <httplib.h>

#include <iostream>
#include <map>
#include <utility>

#include "resim/assert/assert.hh"

namespace resim::testing {

struct ServerHandle {
  httplib::Server server;
};

MockServer::MockServer(std::string host)
    : host_{std::move(host)},
      handle_{std::make_unique<ServerHandle>()} {
  // The default keep alive timeout is 5s, which makes our tests using the mock
  // server excrutiatingly slow. Setting it at 1s somehow overrides any keep
  // alive timeout, so they run much faster.
  handle_->server.set_keep_alive_max_count(1);
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
        res.status = static_cast<int>(HttpResponse::CREATED);
      });
}

MockServer::~MockServer() {
  // Kill the server
  httplib::Client cli{host_, port_};
  cli.Get("/stop");
  server_thread_->join();
}

void MockServer::add_post_receiver(
    const std::string &endpoint,
    PostReceiver &&receiver) {
  handle_->server.Post(
      endpoint,
      [receiver = std::move(receiver)](
          const httplib::Request &req,
          httplib::Response &res,
          const httplib::ContentReader &reader) {
        std::string body;
        reader([&](const char *data, const size_t data_length) {
          body.append(data, data_length);
          return true;
        });
        std::multimap<std::string, std::string> headers;
        for (const auto &header : req.headers) {
          headers.insert(header);
        }
        Response response;
        receiver(headers, body, req.matches, InOut{response});
        res.set_content(response.body, response.mime_type);
        res.status = static_cast<int>(response.status);
      });
}

std::string MockServer::host() const { return host_; }

int MockServer::port() const { return port_; }

void MockServer::listen() {
  port_ = handle_->server.bind_to_any_port(host_);
  server_thread_ = std::make_unique<std::thread>(
      [this]() { handle_->server.listen_after_bind(); });

  // Make sure the server is ready
  {
    httplib::Client cli{host_, port_};
    const httplib::Result res{cli.Get("/ready")};
    const bool got_created =
        res->status == static_cast<int>(HttpResponse::CREATED);
    REASSERT(got_created, "Failed to setup server");
  }
}

}  // namespace resim::testing
