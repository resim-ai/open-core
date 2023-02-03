#pragma once

#include "resim_core/utils/status.hh"
#include "resim_core/visualization/view_update.hh"

namespace resim::visualization {

// ViewClient is an abstract interface for objects which communicate with the
// resim View server. It mainly exists to enable mocking in tests for e.g. the
// resim::visualization::View object.
class ViewClient {
 public:
  ViewClient() = default;
  ViewClient(const ViewClient &) = delete;
  ViewClient(ViewClient &&) = delete;
  ViewClient &operator=(const ViewClient &) = delete;
  ViewClient &operator=(ViewClient &&) = delete;

  // Send the given update to the server.
  virtual Status send_view_update(const ViewUpdate &update) = 0;

  virtual ~ViewClient() = default;
};

}  // namespace resim::visualization
