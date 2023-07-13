#include "resim/simulator/time_lord_update.hh"

#include <gtest/gtest.h>

#include <chrono>

#include "resim/time/timestamp.hh"

namespace resim::simulator {

// This test suite mostly exists to test the comparisons for the TimeLordUpdate
// These are *very important*, as they define the order in which updates occur,
// and *only one update runs per Timestamp*. Changes to these tests should be
// made carefully.
TEST(TimeLordUpdateTest, TimeLordTimeComparison) {
  // SETUP
  TimeLordUpdate update_before{
      time::Timestamp(std::chrono::seconds(0)),
      TimeLordUpdate::UpdateBehaviour::FULL_UPDATE,
      TimeLordUpdate::UpdateType::PRESCHEDULED};

  TimeLordUpdate update_after{
      time::Timestamp(std::chrono::seconds(1)),
      TimeLordUpdate::UpdateBehaviour::FULL_UPDATE,
      TimeLordUpdate::UpdateType::PRESCHEDULED};

  TimeLordUpdate update_after_then_terminate{
      time::Timestamp(std::chrono::seconds(1)),
      TimeLordUpdate::UpdateBehaviour::FULL_UPDATE_THEN_TERMINATE,
      TimeLordUpdate::UpdateType::PRESCHEDULED};

  // VERIFICATION
  EXPECT_EQ(update_before, update_before);
  EXPECT_EQ(update_after, update_after);

  EXPECT_NE(update_before, update_after);
  EXPECT_LE(update_before, update_after);
  EXPECT_LT(update_before, update_after);
  EXPECT_GE(update_after, update_before);
  EXPECT_GT(update_after, update_before);

  EXPECT_NE(update_before, update_after_then_terminate);
  EXPECT_LE(update_before, update_after_then_terminate);
  EXPECT_LT(update_before, update_after_then_terminate);
  EXPECT_GE(update_after_then_terminate, update_before);
  EXPECT_GT(update_after_then_terminate, update_before);
}

TEST(TimeLordUpdateTest, TimeLordBehaviourComparison) {
  // SETUP
  TimeLordUpdate update{
      time::Timestamp(std::chrono::seconds(0)),
      TimeLordUpdate::UpdateBehaviour::FULL_UPDATE,
      TimeLordUpdate::UpdateType::UNIT_SCHEDULED};

  TimeLordUpdate update_then_terminate{
      time::Timestamp(std::chrono::seconds(0)),
      TimeLordUpdate::UpdateBehaviour::FULL_UPDATE_THEN_TERMINATE,
      TimeLordUpdate::UpdateType::PRESCHEDULED};

  TimeLordUpdate unsupported_update{
      time::Timestamp(std::chrono::seconds(0)),
      TimeLordUpdate::UpdateBehaviour::UNSUPPORTED,
      TimeLordUpdate::UpdateType::UNIT_SCHEDULED};

  // VERIFICATION
  EXPECT_NE(update_then_terminate, update);
  EXPECT_LE(update_then_terminate, update);
  EXPECT_LT(update_then_terminate, update);
  EXPECT_GE(update, update_then_terminate);
  EXPECT_GT(update, update_then_terminate);

  EXPECT_NE(update, unsupported_update);
  EXPECT_LE(update, unsupported_update);
  EXPECT_LT(update, unsupported_update);
  EXPECT_GE(unsupported_update, update);
  EXPECT_GT(unsupported_update, update);
}

TEST(TimeLordUpdateTest, TimeLordTypeComparison) {
  // SETUP
  TimeLordUpdate unit_update{
      time::Timestamp(std::chrono::seconds(0)),
      TimeLordUpdate::UpdateBehaviour::FULL_UPDATE,
      TimeLordUpdate::UpdateType::UNIT_SCHEDULED};

  TimeLordUpdate prescheduled_update{
      time::Timestamp(std::chrono::seconds(0)),
      TimeLordUpdate::UpdateBehaviour::FULL_UPDATE,
      TimeLordUpdate::UpdateType::PRESCHEDULED};

  TimeLordUpdate undefined_update{
      time::Timestamp(std::chrono::seconds(0)),
      TimeLordUpdate::UpdateBehaviour::FULL_UPDATE,
      TimeLordUpdate::UpdateType::UNDEFINED};

  // VERIFICATION
  EXPECT_NE(unit_update, prescheduled_update);
  EXPECT_LE(unit_update, prescheduled_update);
  EXPECT_LT(unit_update, prescheduled_update);
  EXPECT_GE(prescheduled_update, unit_update);
  EXPECT_GT(prescheduled_update, unit_update);

  EXPECT_NE(prescheduled_update, undefined_update);
  EXPECT_LE(prescheduled_update, undefined_update);
  EXPECT_LT(prescheduled_update, undefined_update);
  EXPECT_GE(undefined_update, prescheduled_update);
  EXPECT_GT(undefined_update, prescheduled_update);
}
}  // namespace resim::simulator
