// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/time/proto/time_to_proto.hh"

#include <gtest/gtest.h>

#include <random>
#include <ratio>

#include "resim/time/random_duration.hh"

namespace resim::time::proto {

TEST(TestTimeToProto, ConvertTimeToProto) {
  // SETUP
  constexpr int SEED = 932;
  std::mt19937 rng{SEED};
  const time::Timestamp test_timestamp{time::random_duration(InOut{rng})};

  // ACTION
  google::protobuf::Timestamp proto_msg;
  pack(test_timestamp, &proto_msg);

  // VERIFICATION
  EXPECT_EQ(
      proto_msg.seconds() * std::nano::den + proto_msg.nanos(),
      test_timestamp.time_since_epoch().count());
}

TEST(TestTimeToProto, ConvertDurationToProto) {
  // SETUP
  constexpr int SEED = 932;
  std::mt19937 rng{SEED};
  const time::Duration test_duration{time::random_duration(InOut{rng})};

  // ACTION
  google::protobuf::Duration proto_msg;
  pack(test_duration, &proto_msg);

  // VERIFICATION
  EXPECT_EQ(
      proto_msg.seconds() * std::nano::den + proto_msg.nanos(),
      test_duration.count());
}

TEST(TestTimeToProto, RoundTripTimestamp) {
  // SETUP
  constexpr int SEED = 5903111;
  std::mt19937 rng{SEED};
  const time::Timestamp test_timestamp{time::random_duration(InOut{rng})};

  // ACTION
  google::protobuf::Timestamp proto_msg;
  pack(test_timestamp, &proto_msg);
  const time::Timestamp round_trip = unpack(proto_msg);

  // VERIFICATION
  EXPECT_EQ(round_trip, test_timestamp);
}

TEST(TestTimeToProto, RoundTripDuration) {
  // SETUP
  constexpr int SEED = 86001843;
  std::mt19937 rng{SEED};
  const time::Duration test_duration{time::random_duration(InOut{rng})};

  // ACTION
  google::protobuf::Duration proto_msg;
  pack(test_duration, &proto_msg);
  const time::Duration round_trip = unpack(proto_msg);

  // VERIFICATION
  EXPECT_EQ(round_trip, test_duration);
}

}  // namespace resim::time::proto
