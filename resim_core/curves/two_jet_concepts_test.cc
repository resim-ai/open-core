#include "resim_core/curves/two_jet_concepts.hh"

#include <gtest/gtest.h>

#include <string>

#include "resim_core/curves/two_jet.hh"
#include "resim_core/transforms/se3.hh"
#include "resim_core/transforms/so3.hh"

namespace resim::curves {

namespace {
using SE3 = transforms::SE3;
using SO3 = transforms::SO3;
}  // namespace

// Test that we can correctly identify TwoJets.
TEST(TwoJetTraitsTest, TestIsTwoJet) {
  // SETUP / ACTION / VERIFICATION
  static_assert(TwoJetType<TwoJetL<SE3>>);
  static_assert(TwoJetType<TwoJetL<SO3>>);
  static_assert(TwoJetType<TwoJetR<SE3>>);
  static_assert(TwoJetType<TwoJetR<SO3>>);
  static_assert(!TwoJetType<int>);
  static_assert(!TwoJetType<std::string>);
  static_assert(!TwoJetType<SE3>);
}

}  // namespace resim::curves
