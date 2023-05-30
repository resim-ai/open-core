#include "resim_core/utils/testing/mock_logger.hh"

#include "gtest/gtest.h"
#include "resim_core/assert/assert.hh"
#include "resim_core/time/timestamp.hh"
#include "resim_core/transforms/proto/se3.pb.h"
#include "resim_core/transforms/proto/so3.pb.h"
#include "resim_core/utils/proto/uuid.pb.h"
#include "resim_core/utils/proto/uuid_to_proto.hh"
#include "resim_core/utils/uuid.hh"

namespace resim::testing {

namespace {
const UUID UUID1{UUID::new_uuid()};
const UUID UUID2{UUID::new_uuid()};

const time::Timestamp T1{std::chrono::seconds(1)};
const time::Timestamp T2{T1 + std::chrono::nanoseconds(1)};
const std::string UUID_CHANNEL = "UUID";
const std::string SE3_CHANNEL = "SE3";
}  // namespace

TEST(MockLoggerTest, Log) {
  // SETUP
  MockLogger::ChannelToMessageMap msg_map;
  MockLogger logger(msg_map);

  proto::UUID uuid1_msg{};
  proto::UUID uuid2_msg{};
  resim::proto::pack(UUID1, &uuid1_msg);
  resim::proto::pack(UUID2, &uuid2_msg);

  // ACTION
  logger.add_proto_channel<proto::UUID>(UUID_CHANNEL);
  logger.log_proto(UUID_CHANNEL, T1, uuid1_msg);
  logger.log_proto(UUID_CHANNEL, T2, uuid2_msg);

  // VERIFY
  EXPECT_TRUE(msg_map.contains(UUID_CHANNEL));
  EXPECT_TRUE(msg_map.at(UUID_CHANNEL).size() == 2U);

  EXPECT_TRUE(msg_map.at(UUID_CHANNEL).at(0).time == T1);
  EXPECT_TRUE(
      msg_map.at(UUID_CHANNEL).at(0).message == uuid1_msg.SerializeAsString());

  EXPECT_TRUE(msg_map.at(UUID_CHANNEL).at(1).time == T2);
  EXPECT_TRUE(
      msg_map.at(UUID_CHANNEL).at(1).message == uuid2_msg.SerializeAsString());
}

TEST(MockLoggerTest, LogWithoutChannel) {
  // SETUP
  MockLogger::ChannelToMessageMap msg_map;
  MockLogger logger(msg_map);

  proto::UUID uuid_msg;

  // ACTION + VERIFY

  EXPECT_THROW(
      logger.log_proto(UUID_CHANNEL, time::Timestamp(), uuid_msg),
      AssertException);
}

TEST(MockLoggerTest, BadChannelAdd) {
  // SETUP
  MockLogger::ChannelToMessageMap msg_map;
  MockLogger logger(msg_map);

  transforms::proto::SE3 se3_msg;
  transforms::proto::SO3 so3_msg;
  logger.add_proto_channel<transforms::proto::SE3>(SE3_CHANNEL);

  // ACTION + VERIFY
  EXPECT_NO_THROW(
      logger.add_proto_channel<transforms::proto::SE3>(SE3_CHANNEL));
  EXPECT_THROW(
      logger.add_proto_channel<transforms::proto::SO3>(SE3_CHANNEL),
      AssertException);
}

TEST(MockLoggerTest, BadLogType) {
  // SETUP
  MockLogger::ChannelToMessageMap msg_map;
  MockLogger logger(msg_map);

  transforms::proto::SE3 se3_msg;
  transforms::proto::SO3 so3_msg;
  logger.add_proto_channel<transforms::proto::SE3>(SE3_CHANNEL);

  // ACTION + VERIFY
  EXPECT_NO_THROW(logger.log_proto(SE3_CHANNEL, time::Timestamp(), se3_msg));
  EXPECT_THROW(
      logger.log_proto(SE3_CHANNEL, time::Timestamp(), so3_msg),
      AssertException);
}

}  // namespace resim::testing
