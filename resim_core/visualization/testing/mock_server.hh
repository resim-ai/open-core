
#pragma once

#include "resim_core/testing/mock_server.hh"
#include "resim_core/utils/http_response.hh"
#include "resim_core/utils/uuid.hh"
#include "resim_core/visualization/client/proto/view_client.pb.h"
#include "resim_core/visualization/proto/view_update.pb.h"
#include "resim_core/visualization/proto/view_update_to_proto.hh"
#include "resim_core/visualization/view_update.hh"

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

 private:
  Receiver receiver_;
  HttpResponse view_update_response_code_{HttpResponse::CREATED};
  UUID session_id_;

  ::resim::testing::MockServer server_;
};

}  // namespace resim::visualization::testing
