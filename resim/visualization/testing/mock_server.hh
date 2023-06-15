
#pragma once

#include "resim/testing/mock_server.hh"
#include "resim/utils/http_response.hh"
#include "resim/utils/uuid.hh"
#include "resim/visualization/client/proto/view_client.pb.h"
#include "resim/visualization/proto/view_update.pb.h"
#include "resim/visualization/proto/view_update_to_proto.hh"
#include "resim/visualization/view_update.hh"

namespace resim::visualization::testing {

// Runs a mock server that acts like the view API. This server picks an
// arbitrary port on the given host.
class MockServer {
 public:
  using Receiver = std::function<client::proto::ViewSessionUpdateResponse(
      const ViewUpdate &,
      const UUID &,
      uint64_t)>;

  // Constructor:
  // @param[in] host - The host to serve on.
  // @param[in] session_id - The session id to use.
  // @param[in] receiver - A function to call with the received ViewUpdate,
  //                       session id, and update id on receipt from a client.
  // @param[in] view_update_response_code - What code we should return
  //                                        when the client posts a
  //                                        view update.
  MockServer(
      std::string host,
      const UUID &session_id,
      Receiver &&receiver,
      HttpResponse view_update_response_code = HttpResponse::CREATED);

  MockServer(const MockServer &) = delete;
  MockServer(MockServer &&) = delete;
  MockServer &operator=(const MockServer &) = delete;
  MockServer &operator=(MockServer &&) = delete;

  ~MockServer() = default;

  // Getters for the host and port on which we're serving.
  std::string host() const;
  int port() const;

  // Returns a valid JWT for authorization. Note that this is the ONLY
  // token that the server will accept.
  static std::string valid_token() { return VALID_TOKEN_; }
  // Returns a token that will be rejected as Unauthorized (401).
  // An unauthorized token could be expired, malformed, or otherwise invalid.
  static std::string unauthorized_token() { return UNAUTHORIZED_TOKEN_; }
  // Returns a valid token that does not have permission to the endpoint
  // (for example, insufficient scopes).
  static std::string forbidden_token() { return FORBIDDEN_TOKEN_; }

 private:
  static const std::string VALID_TOKEN_;
  static const std::string UNAUTHORIZED_TOKEN_;
  static const std::string FORBIDDEN_TOKEN_;

  Receiver receiver_;
  HttpResponse view_update_response_code_{HttpResponse::CREATED};
  UUID session_id_;

  ::resim::testing::MockServer server_;
};

}  // namespace resim::visualization::testing
