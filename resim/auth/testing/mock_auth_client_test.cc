// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/auth/testing/mock_auth_client.hh"

#include <gtest/gtest.h>

namespace resim::auth::testing {

const std::string TEST_TOKEN = "test token";
const std::string REFRESH_TOKEN = "refresh token";

TEST(MockAuthClientTest, TestGetJwt) {
  MockAuthClient client(TEST_TOKEN);
  EXPECT_EQ(client.get_jwt(), TEST_TOKEN);
  EXPECT_NO_THROW(client.refresh());
  EXPECT_EQ(client.get_jwt(), TEST_TOKEN);
}

TEST(MockAuthClientTest, TestRefresh) {
  MockAuthClient client(TEST_TOKEN, REFRESH_TOKEN);
  EXPECT_EQ(client.get_jwt(), TEST_TOKEN);
  EXPECT_NO_THROW(client.refresh());
  EXPECT_EQ(client.get_jwt(), REFRESH_TOKEN);
}
}  // namespace resim::auth::testing
