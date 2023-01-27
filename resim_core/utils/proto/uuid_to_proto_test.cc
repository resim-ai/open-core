
#include "resim_core/utils/proto/uuid_to_proto.hh"

#include <gtest/gtest.h>

#include "resim_core/utils/proto/uuid.pb.h"
#include "resim_core/utils/uuid.hh"

namespace resim {

TEST(UUIDToProtoTest, TestPack) {
  // SETUP
  proto::UUID msg;
  const UUID uuid{UUID::new_uuid()};

  // ACTION
  pack(uuid, &msg);

  // VERIFICATION
  EXPECT_EQ(msg.data(), uuid.to_string());
}

TEST(UUIDToProtoTest, TestRoundTrip) {
  // SETUP
  proto::UUID msg;
  const UUID uuid{UUID::new_uuid()};

  // ACTION
  pack(uuid, &msg);
  const UUID other_uuid{unpack(msg)};

  // VERIFICATION
  EXPECT_EQ(uuid, other_uuid);
}

TEST(UUIDToProtoDeathTest, TestPackInvalid) {
  // ACTION / VERIFICATION
  EXPECT_DEATH(
      proto::pack(UUID::new_uuid(), nullptr),
      "Can't pack into invalid proto!");
}

}  // namespace resim
