// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <filesystem>
#include <string>

#include "resim/auth/auth_client_interface.hh"
#include "resim/utils/status.hh"
#include "resim/visualization/proto/view_update.pb.h"
#include "resim/visualization/view_client_interface.hh"

namespace resim::visualization {

// A simple client that packs SE3 and sends to server.
class ViewClient : public ViewClientInterface {
 public:
  explicit ViewClient(std::string base_url);
  Status send_view_update(const ViewUpdate &update) override;

  // Allows overriding the auth client for testing purposes.
  void set_auth_client(std::unique_ptr<auth::AuthClientInterface> auth_client);

  // determine_token_root expects to be passed getenv("HOME") as its parameter.
  // It can't just get the variable itself because we can't test the branch
  // where HOME is defined; bazel test doesn't define it and there's no way to
  // set environment variables in C++. If given `nullptr` as its argument (which
  // is the case when HOME is undefined) it falls back to TEST_TMPDIR, which is
  // defined under bazel test. It probably does something weird if neither
  // variable is defined, but unfortunately we can't write code that handles
  // that case because there's no way to test it. This entire setup is a
  // complete mess that only exists to push up codecov numbers. Requiring 100%
  // coverage is an antipattern.
  //
  // Exposed for testing only.
  static std::filesystem::path determine_token_root(const char *home);

 private:
  // Log a url pointing to the session-specific page in the resim app.
  static void log_app_url(const std::string &url);

  std::unique_ptr<auth::AuthClientInterface> auth_client_;
  std::string base_url_;    // base URL for endpoint
  std::string session_id_;  // unique ID per user
  uint64_t update_id_ = 0;  // view update counter
};

}  // namespace resim::visualization
