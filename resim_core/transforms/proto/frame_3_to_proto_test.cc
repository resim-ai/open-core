#include "resim_core/transforms/proto/frame_3_to_proto.hh"

#include <gtest/gtest.h>

#include "resim_core/assert/assert.hh"
#include "resim_core/transforms/frame.hh"
#include "resim_core/transforms/proto/frame_3.pb.h"

namespace resim::transforms {

TEST(Frame3ToProtoTest, Pack) {
  // SETUP
  proto::Frame_3 msg;
  const auto frame = Frame<3>::new_frame();

  // ACTION
  pack(frame, &msg);

  // VERIFICATION
  EXPECT_EQ(msg.id().data(), frame.id().to_string());
}

TEST(Frame3ToProtoTest, RoundTrip) {
  // SETUP
  proto::Frame_3 msg;
  const auto frame = Frame<3>::new_frame();

  // ACTION
  pack(frame, &msg);
  const auto retrieved_frame = unpack(msg);

  // VERIFICATION
  EXPECT_EQ(retrieved_frame.DIMS, 3);
  EXPECT_EQ(retrieved_frame, frame);
}

TEST(Frame3ToProtoDeathTest, TestPackInvalid) {
  // ACTION / VERIFICATION
  EXPECT_THROW(proto::pack(Frame<3>::new_frame(), nullptr), AssertException);
}

}  // namespace resim::transforms
