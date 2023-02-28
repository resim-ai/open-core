#include "resim_core/utils/uuid.hh"

#include <gtest/gtest.h>
namespace resim {

namespace {

constexpr unsigned int ARRAY_SIZE = UUID::ARRAY_SIZE;

}  // namespace

// Basic test to make sure the linux uuid lib is correctly installed and linked.
// Generates two raw uuid_t objects and verifies that they are different.
TEST(RawUUIDTest, GenerateUuids) {
  uuid_t one;
  uuid_t two;
  // NOLINTBEGIN(cppcoreguidelines-pro-bounds-array-to-pointer-decay)
  uuid_generate(one);
  uuid_generate(two);
  EXPECT_NE(uuid_compare(one, two), 0);
  // NOLINTEND(cppcoreguidelines-pro-bounds-array-to-pointer-decay)
}

// Basic test to make sure the linux uuid lib is correctly installed and linked.
// Copies one raw uuid_t to another and verifies that they are the same.
TEST(RawUUIDTest, CopyUuids) {
  uuid_t one;
  uuid_t two;
  // NOLINTBEGIN(cppcoreguidelines-pro-bounds-array-to-pointer-decay)
  uuid_generate(one);
  uuid_copy(two, one);
  EXPECT_EQ(uuid_compare(one, two), 0);
  // NOLINTEND(cppcoreguidelines-pro-bounds-array-to-pointer-decay)
}

TEST(UUIDTest, ConstructEmptyUUID) {
  const UUID empty_uuid;
  const std::array<unsigned char, ARRAY_SIZE> null_id{{0}};
  EXPECT_EQ(empty_uuid.id(), null_id);
}

TEST(UUIDTest, ConstructUUID) {
  std::array<unsigned char, ARRAY_SIZE> id{};
  uuid_generate(id.data());
  UUID uuid(id);
  EXPECT_EQ(uuid.id(), id);
  const std::array<unsigned char, ARRAY_SIZE> null_id{{0}};
  EXPECT_NE(uuid.id(), null_id);
}

TEST(UUIDTest, NewUUID) {
  UUID uuid_a = UUID::new_uuid();
  UUID uuid_b = UUID::new_uuid();
  EXPECT_NE(uuid_a.id(), uuid_b.id());
  EXPECT_NE(uuid_a, uuid_b);
}

TEST(UUIDTest, CopyConstructUUID) {
  UUID uuid_a = UUID::new_uuid();
  UUID uuid_b(uuid_a);
  EXPECT_EQ(uuid_a.id(), uuid_b.id());
  EXPECT_EQ(uuid_a, uuid_b);
}

TEST(UUIDTest, TestParseRoundTrip) {
  // SETUP
  const std::string ascii_rep{"d8c2276f-fffc-4e49-bba7-4254a0b0faf3"};

  // ACTION
  const UUID uuid{ascii_rep};
  const std::string test_rep{uuid.to_string()};

  // VERIFICATION
  EXPECT_EQ(ascii_rep, test_rep);
}

TEST(UUIDTest, CopyAssignUUID) {
  UUID uuid_a = UUID::new_uuid();
  UUID uuid_b = uuid_a;
  EXPECT_EQ(uuid_a.id(), uuid_b.id());
  EXPECT_EQ(uuid_a, uuid_b);
}

TEST(UUIDTest, UUIDBooleanOperators) {
  UUID uuid_a = UUID::new_uuid();
  UUID uuid_b = uuid_a;
  UUID uuid_c = UUID::new_uuid();
  EXPECT_EQ(uuid_a, uuid_b);
  EXPECT_NE(uuid_a, uuid_c);
  EXPECT_NE(uuid_b, uuid_c);
}

}  // namespace resim
