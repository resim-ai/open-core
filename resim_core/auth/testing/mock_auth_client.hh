#pragma once

#include "resim_core/auth/auth_client.hh"

namespace resim::auth {

// MockAuthClient is a very simple mock for the AuthClient that returns a hard
// coded fake JWT.
class MockAuthClient : public AuthClientInterface {
 public:
  std::string get_jwt() override { return "test jwt"; }

  void refresh() override {}
};

}  // namespace resim::auth
