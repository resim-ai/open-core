#include "resim_core/auth/testing/mock_auth_client.hh"

#include <gtest/gtest.h>

namespace resim::auth::testing {

TEST(MockAuthClientTest, TestGetJwt) {
  MockAuthClient client;
  std::string jwt = client.get_jwt();
  EXPECT_FALSE(jwt.empty());
}

TEST(MockAuthClientTest, TestRefresh) {
  MockAuthClient client;
  std::string jwt = client.get_jwt();
  EXPECT_FALSE(jwt.empty());

  EXPECT_NO_THROW(client.refresh());

  EXPECT_FALSE(jwt.empty());
}
}  // namespace resim::auth::testing
