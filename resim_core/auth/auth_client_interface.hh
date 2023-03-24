#pragma once

#include <string>

namespace resim::auth {
// Abstract interface for an AuthClient.
class AuthClientInterface {
 public:
  AuthClientInterface() = default;
  AuthClientInterface(const AuthClientInterface &) = default;
  AuthClientInterface(AuthClientInterface &&) = default;
  AuthClientInterface &operator=(const AuthClientInterface &) = default;
  AuthClientInterface &operator=(AuthClientInterface &&) = default;
  virtual ~AuthClientInterface() = default;

  // Fetch the current token, potentially querying an auth server for
  // it.
  virtual std::string get_jwt() = 0;

  // Refresh the current token. Users should call this if the current
  // token becomes stale, although they may refresh preemptively if
  // they desire.
  virtual void refresh() = 0;
};

}  // namespace resim::auth
