
#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "resim_core/utils/uuid.hh"
#include "resim_core/visualization/client/proto/view_client.pb.h"
#include "resim_core/visualization/proto/view_update.pb.h"
#include "resim_core/visualization/proto/view_update_to_proto.hh"
#include "resim_core/visualization/view_update.hh"

namespace resim::visualization::testing {

// Forward declaration of a handle for the server, which we use the PIMPL idiom
// fore because httplib.h, which we require, pollutes the global namespace.
struct ServerHandle;

// Runs a mock server that acts like the view API. This server picks an
// arbitrary port on the given host.
class MockServer {
 public:
  enum class ResponseCode {
    CREATED = 201,
    NOT_FOUND = 404,
  };

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
      ResponseCode view_update_response_code = ResponseCode::CREATED);

  MockServer(const MockServer &) = delete;
  MockServer(MockServer &&) = delete;
  MockServer &operator=(const MockServer &) = delete;
  MockServer &operator=(MockServer &&) = delete;

  // Getters for the host and port on which we're serving.
  std::string host() const;
  int port() const;

  ~MockServer();

 private:
  std::string host_;
  int port_ = 0;
  Receiver receiver_;
  ResponseCode view_update_response_code_{ResponseCode::CREATED};

  std::unique_ptr<ServerHandle> handle_;
  UUID session_id_;

  // A thread on which the server listens
  std::unique_ptr<std::thread> server_thread_;
};

}  // namespace resim::visualization::testing
