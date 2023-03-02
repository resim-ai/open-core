
#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <regex>
#include <string>
#include <thread>

#include "resim_core/utils/http_response.hh"
#include "resim_core/utils/inout.hh"

namespace resim::testing {

// Forward declaration of a handle for the server, which we use the PIMPL idiom
// fore because httplib.h, which we require, pollutes the global namespace.
struct ServerHandle;

// Runs a mock server for testing purposes. This server picks an
// arbitrary port on the given host. Users can add their own custom
// endpoints to it for whichever specific test they're writing.
class MockServer {
 public:
  // Constructor:
  // @param[in] host - The host to serve on.
  explicit MockServer(std::string host);

  MockServer(const MockServer &) = delete;
  MockServer(MockServer &&) = delete;
  MockServer &operator=(const MockServer &) = delete;
  MockServer &operator=(MockServer &&) = delete;

  ~MockServer();

  // Getters for the host and port on which we're serving.
  std::string host() const;
  int port() const;

  // A struct to populate with response information.
  struct Response {
    std::string body;
    std::string mime_type;
    HttpResponse status{HttpResponse::INVALID};
  };

  // A receiver function which is called with (in order), the request body, a
  // std::smatch containing matches from the endpoint (specified to
  // add_post_receiver() below as a regex), and an InOut for the response which
  // this receiver can populate.
  using PostReceiver = std::function<
      void(const std::string &, const std::smatch &, InOut<Response>)>;

  // Add a PostReceiver function (as documented above) to handle endpoints
  // matching the given endpoint regular expression expressed as a string.
  void add_post_receiver(const std::string &endpoint, PostReceiver &&receiver);

  // Begin listening.
  void listen();

 private:
  std::string host_;
  int port_ = 0;

  std::unique_ptr<ServerHandle> handle_;

  // A thread on which the server listens
  std::unique_ptr<std::thread> server_thread_;
};

}  // namespace resim::testing
