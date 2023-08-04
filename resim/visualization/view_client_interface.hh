// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include "resim/utils/status.hh"
#include "resim/visualization/view_update.hh"

namespace resim::visualization {

// ViewClient is an abstract interface for objects which communicate with the
// resim View server. It mainly exists to enable mocking in tests for e.g. the
// resim::visualization::View object.
class ViewClientInterface {
 public:
  ViewClientInterface() = default;
  ViewClientInterface(const ViewClientInterface &) = delete;
  ViewClientInterface(ViewClientInterface &&) = delete;
  ViewClientInterface &operator=(const ViewClientInterface &) = delete;
  ViewClientInterface &operator=(ViewClientInterface &&) = delete;

  // Send the given update to the server.
  virtual Status send_view_update(const ViewUpdate &update) = 0;

  virtual ~ViewClientInterface() = default;
};

}  // namespace resim::visualization
