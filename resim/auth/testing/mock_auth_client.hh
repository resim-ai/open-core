// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include "resim/auth/auth_client_interface.hh"

namespace resim::auth {

// MockAuthClient is a very simple mock for the AuthClient that returns a hard
// coded fake JWT.
class MockAuthClient : public AuthClientInterface {
 public:
  explicit MockAuthClient(std::string token, std::string refresh = "")
      : token_(std::move(token)),
        refresh_(refresh.empty() ? token_ : std::move(refresh)) {}

  std::string get_jwt() override { return token_; }
  void refresh() override { token_ = refresh_; }

 private:
  std::string token_;
  const std::string refresh_;
};

}  // namespace resim::auth
