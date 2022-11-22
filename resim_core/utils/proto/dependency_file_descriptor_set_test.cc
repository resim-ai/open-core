
#include "resim_core/utils/proto/dependency_file_descriptor_set.hh"

#include <google/protobuf/descriptor.pb.h>
#include <gtest/gtest.h>

#include <unordered_set>

#include "resim_core/utils/proto/testing/message_a.pb.h"
#include "resim_core/utils/proto/testing/message_b.pb.h"
#include "resim_core/utils/proto/testing/message_c.pb.h"
#include "resim_core/utils/proto/testing/message_d.pb.h"

namespace resim {

namespace {

// A simple helper to add a file descriptor for the given ProtoT to the given
// FileDescriptorSet.
template <class ProtoT>
void add_file_descriptor(google::protobuf::FileDescriptorSet& fds) {
  ProtoT::GetDescriptor()->file()->CopyTo(fds.add_file());
}

}  // namespace

TEST(DepdendencyFileDescriptorFileSetTest, TestMakeDependencySet) {
  // ACTION
  const std::string result = dependency_file_descriptor_set(
      *proto::testing::MessageD::GetDescriptor());

  // VERIFICATION
  google::protobuf::FileDescriptorSet descriptor_set;
  // Verify parse-ability
  ASSERT_TRUE(descriptor_set.ParseFromString(result));

  // Verify the size
  EXPECT_EQ(descriptor_set.file().size(), 4U);

  // Verify uniqueness
  std::unordered_set<std::string> observed;
  for (const auto& file : descriptor_set.file()) {
    EXPECT_TRUE(observed.insert(file.name()).second);
  }
  // Verify that we have the expected elements
  EXPECT_TRUE(
      observed.contains("resim_core/utils/proto/testing/message_a.proto"));
  EXPECT_TRUE(
      observed.contains("resim_core/utils/proto/testing/message_b.proto"));
  EXPECT_TRUE(
      observed.contains("resim_core/utils/proto/testing/message_c.proto"));
  EXPECT_TRUE(
      observed.contains("resim_core/utils/proto/testing/message_d.proto"));
}
}  // namespace resim
