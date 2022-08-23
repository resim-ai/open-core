#include "resim_core/curves/two_jet.hh"

#include <gtest/gtest.h>

#include "resim_core/transforms/framed_group.hh"
#include "resim_core/transforms/se3.hh"
#include "resim_core/transforms/so3.hh"

namespace resim::curves {

namespace {
using SE3 = transforms::SE3;
using SO3 = transforms::SO3;
using FSE3 = transforms::FSE3;
using FSO3 = transforms::FSO3;
}  // namespace

template <typename T>
class TwoJetTests : public ::testing::Test {
 protected:
  void SetUp() override {
    // TODO(https://app.asana.com/0/1202178773526279/1202774361010794/f)
    constexpr unsigned int SEED = 31;
    srand(SEED);
  }

  void TearDown() override { srand(1); }

  typename T::TangentVector test_vector() { return T::TangentVector::Random(); }

  TwoJet<T> test_two_jet() {
    return TwoJet<T>(T::exp(test_vector()), test_vector(), test_vector());
  }
};

using LieGroupTypes = ::testing::Types<SE3, SO3, FSE3, FSO3>;
TYPED_TEST_SUITE(TwoJetTests, LieGroupTypes);

template <typename T>
class UnframedTwoJetTests : public TwoJetTests<T> {};

using UnframedTypes = ::testing::Types<SE3, SO3>;
TYPED_TEST_SUITE(UnframedTwoJetTests, UnframedTypes);

namespace {
// For each test below we employ (deterministic) randomly generated TwoJet
// objects. We desire to test a few different generated TwoJets in order to
// confirm the implementation is robust. This number should be more than 1.
// However, it does not need to be hundreds, because we are testing
// fundamental functionality which should not be susceptible to errors of high
// sensitivity. Seven is a (hopefully) lucky guess at the 'right' number.
constexpr unsigned int NUM_TRIES = 7;
}  // namespace

TYPED_TEST(TwoJetTests, InverseRoundTrip) {
  for (unsigned int i = 0; i < NUM_TRIES; ++i) {
    TwoJet<TypeParam> test_tj = this->test_two_jet();
    EXPECT_FALSE(test_tj.is_approx(test_tj.inverse()));
    EXPECT_TRUE(test_tj.is_approx(test_tj.inverse().inverse()));
  }
}

TYPED_TEST(TwoJetTests, CompositionByInverseIsIdentityAndZeros) {
  for (unsigned int i = 0; i < NUM_TRIES; ++i) {
    TwoJet<TypeParam> test_tj = this->test_two_jet();
    TwoJet<TypeParam> id_tj = TwoJet<TypeParam>::identity();
    TwoJet<TypeParam> expected_id_tj = test_tj.inverse() * test_tj;
    EXPECT_TRUE(
        expected_id_tj.frame_from_ref().is_approx(id_tj.frame_from_ref()));
    EXPECT_TRUE(expected_id_tj.d_frame_from_ref().isZero());
    EXPECT_TRUE(expected_id_tj.d2_frame_from_ref().isZero());
  }
}

TYPED_TEST(UnframedTwoJetTests, Associativity) {
  for (unsigned int i = 0; i < NUM_TRIES; ++i) {
    TwoJet<TypeParam> a_from_b = this->test_two_jet();
    TwoJet<TypeParam> b_from_c = this->test_two_jet();
    TwoJet<TypeParam> c_from_d = this->test_two_jet();
    TwoJet<TypeParam> a_from_d_0 = (a_from_b * b_from_c) * c_from_d;
    TwoJet<TypeParam> a_from_d_1 = a_from_b * (b_from_c * c_from_d);
    EXPECT_TRUE(a_from_d_0.is_approx(a_from_d_1));
  }
}

TYPED_TEST(UnframedTwoJetTests, Composition) {
  for (unsigned int i = 0; i < NUM_TRIES; ++i) {
    TwoJet<TypeParam> a_from_b = this->test_two_jet();
    TwoJet<TypeParam> b_from_c = this->test_two_jet();
    TwoJet<TypeParam> a_from_c = a_from_b * b_from_c;
    EXPECT_FALSE(a_from_c.is_approx(a_from_b));
    TwoJet<TypeParam> test_a_from_b = a_from_c * b_from_c.inverse();
    EXPECT_TRUE(test_a_from_b.is_approx(a_from_b));
    TwoJet<TypeParam> test_b_from_c = a_from_b.inverse() * a_from_c;
    EXPECT_TRUE(test_b_from_c.is_approx(b_from_c));
  }
}

TYPED_TEST(TwoJetTests, IsApproxTest) {
  TwoJet<TypeParam> test_two_jet_0 = this->test_two_jet();
  TwoJet<TypeParam> test_two_jet_1 = this->test_two_jet();
  EXPECT_FALSE(test_two_jet_0.is_approx(test_two_jet_1));
  test_two_jet_0.set_frame_from_ref(test_two_jet_1.frame_from_ref());
  EXPECT_FALSE(test_two_jet_0.is_approx(test_two_jet_1));
  test_two_jet_0.set_d_frame_from_ref(test_two_jet_1.d_frame_from_ref());
  EXPECT_FALSE(test_two_jet_0.is_approx(test_two_jet_1));
  test_two_jet_0.set_d2_frame_from_ref(test_two_jet_1.d2_frame_from_ref());
  EXPECT_TRUE(test_two_jet_0.is_approx(test_two_jet_1));
}

}  // namespace resim::curves
