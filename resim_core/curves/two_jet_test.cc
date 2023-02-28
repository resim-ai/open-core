#include "resim_core/curves/two_jet.hh"

#include <gtest/gtest.h>

#include <random>

#include "resim_core/curves/two_jet_concepts.hh"
#include "resim_core/curves/two_jet_test_helpers.hh"
#include "resim_core/testing/random_matrix.hh"
#include "resim_core/transforms/framed_group.hh"
#include "resim_core/transforms/liegroup_concepts.hh"
#include "resim_core/transforms/se3.hh"
#include "resim_core/transforms/so3.hh"

namespace resim::curves {

namespace {
using SE3 = transforms::SE3;
using SO3 = transforms::SO3;
using FSE3 = transforms::FSE3;
using FSO3 = transforms::FSO3;

// For each test below we employ (deterministic) randomly generated TwoJet
// objects. We desire to test a few different generated TwoJets in order to
// confirm the implementation is robust. This number should be more than 1.
// However, it does not need to be hundreds, because we are testing
// fundamental functionality which should not be susceptible to errors of high
// sensitivity. Seven is a (hopefully) lucky guess at the 'right' number.
constexpr unsigned int NUM_TRIES = 7;

// An explicit seed for deterministic generation of test objects.
constexpr unsigned int SEED = 11;

}  // namespace

// TwoJetL and TwoJetR implement a number of common methods (as enfoced in the
// TwoJetType concept). Therefore some tests are also common these are
// implemented below and templated to the TwoJetL and TwoJetR types.
template <TwoJetType TwoJet>
class TwoJetTestsBase : public ::testing::Test {
 protected:
  void SetUp() override { tj_helper_ = TwoJetTestHelper<TwoJet>(SEED); }
  TwoJetTestHelper<TwoJet> &tj_helper() { return tj_helper_; }

 private:
  TwoJetTestHelper<TwoJet> tj_helper_;
};

template <typename T>
class TwoJetCommonTests : public TwoJetTestsBase<T> {};

using TwoJetTypes = ::testing::Types<
    TwoJetL<SE3>,
    TwoJetL<SO3>,
    TwoJetL<FSE3>,
    TwoJetL<FSO3>,
    TwoJetR<SE3>,
    TwoJetR<SO3>,
    TwoJetR<FSE3>,
    TwoJetR<FSO3>>;
TYPED_TEST_SUITE(TwoJetCommonTests, TwoJetTypes);

template <typename T>
class UnframedTwoJetCommonTests : public TwoJetCommonTests<T> {};

using UnframedTwoJetTypes =
    ::testing::Types<TwoJetL<SE3>, TwoJetL<SO3>, TwoJetR<SE3>, TwoJetR<SO3>>;
TYPED_TEST_SUITE(UnframedTwoJetCommonTests, UnframedTwoJetTypes);

TYPED_TEST(TwoJetCommonTests, InverseRoundTrip) {
  std::vector<TypeParam> test_elements =
      TestFixture::tj_helper().make_test_two_jet_elements(NUM_TRIES);
  for (const TypeParam &test_tj : test_elements) {
    EXPECT_TRUE(
        test_tj.is_approx(TypeParam::identity()) ||
        !test_tj.is_approx(test_tj.inverse()));
    EXPECT_TRUE(test_tj.is_approx(test_tj.inverse().inverse()));
  }
}

TYPED_TEST(UnframedTwoJetCommonTests, Associativity) {
  for (unsigned int i = 0; i < NUM_TRIES; ++i) {
    TypeParam a_from_b = TestFixture::tj_helper().make_test_two_jet();
    TypeParam b_from_c = TestFixture::tj_helper().make_test_two_jet();
    TypeParam c_from_d = TestFixture::tj_helper().make_test_two_jet();
    TypeParam a_from_d_0 = (a_from_b * b_from_c) * c_from_d;
    TypeParam a_from_d_1 = a_from_b * (b_from_c * c_from_d);
    EXPECT_TRUE(a_from_d_0.is_approx(a_from_d_1));
  }
}

TYPED_TEST(UnframedTwoJetCommonTests, Composition) {
  for (unsigned int i = 0; i < NUM_TRIES; ++i) {
    TypeParam a_from_b = TestFixture::tj_helper().make_test_two_jet();
    TypeParam b_from_c = TestFixture::tj_helper().make_test_two_jet();
    TypeParam a_from_c = a_from_b * b_from_c;

    EXPECT_FALSE(a_from_c.is_approx(a_from_b));
    TypeParam test_a_from_b = a_from_c * b_from_c.inverse();
    EXPECT_TRUE(test_a_from_b.is_approx(a_from_b));
    TypeParam test_b_from_c = a_from_b.inverse() * a_from_c;
    EXPECT_TRUE(test_b_from_c.is_approx(b_from_c));
  }
}

// TwoJetL specific tests. These tests use methods only defined in TwoJetL
// and must be tested separately.
template <transforms::LieGroupType Group>
class TwoJetLTests : public TwoJetTestsBase<TwoJetL<Group>> {
 protected:
  TwoJetL<Group> extrapolate(const TwoJetL<Group> &x, const double dt) {
    constexpr double HALF = 0.5;
    return TwoJetL<Group>{
        Group::exp(
            dt * (x.d_frame_from_ref() + HALF * dt * x.d2_frame_from_ref())) *
            x.frame_from_ref(),
        dt * x.d2_frame_from_ref() + x.d_frame_from_ref(),
        x.d2_frame_from_ref()};
  }
};

using LieGroupTypes = ::testing::Types<SE3, SO3, FSE3, FSO3>;
TYPED_TEST_SUITE(TwoJetLTests, LieGroupTypes);

template <typename T>
class UnframedTwoJetLTests : public TwoJetLTests<T> {};

using UnframedTypes = ::testing::Types<SE3, SO3>;
TYPED_TEST_SUITE(UnframedTwoJetLTests, UnframedTypes);

TYPED_TEST(TwoJetLTests, CompositionByInverseIsIdentityAndZeros) {
  std::vector<TwoJetL<TypeParam>> test_elements =
      TestFixture::tj_helper().make_test_two_jet_elements(NUM_TRIES);
  for (const TwoJetL<TypeParam> &test_tj : test_elements) {
    TwoJetL<TypeParam> id_tj = TwoJetL<TypeParam>::identity();
    TwoJetL<TypeParam> expected_id_tj = test_tj.inverse() * test_tj;
    constexpr double TOLERANCE = 1e-9;
    EXPECT_TRUE(
        expected_id_tj.frame_from_ref().is_approx(id_tj.frame_from_ref()));
    EXPECT_TRUE(expected_id_tj.d_frame_from_ref().isZero());
    EXPECT_TRUE(expected_id_tj.d2_frame_from_ref().isZero(TOLERANCE));
  }
}

TYPED_TEST(UnframedTwoJetLTests, NumericalDerivativesTest) {
  for (unsigned int i = 0; i < NUM_TRIES; ++i) {
    TwoJetL<TypeParam> a_from_b = TestFixture::tj_helper().make_test_two_jet();
    TwoJetL<TypeParam> b_from_c = TestFixture::tj_helper().make_test_two_jet();
    TwoJetL<TypeParam> a_from_c = a_from_b * b_from_c;

    constexpr double h = 1e-8;
    auto a_from_b_p = this->extrapolate(a_from_b, h);
    auto b_from_c_p = this->extrapolate(b_from_c, h);
    auto a_from_c_p = a_from_b_p * b_from_c_p;

    typename TypeParam::TangentVector expected{
        (a_from_c_p.d_frame_from_ref() - a_from_c.d_frame_from_ref()) / h};
    // Error is expected to be somewhat larger than the step h, so order of
    // magnitude is used as the threshold.
    constexpr double OOM = 10.;
    EXPECT_LT((expected - a_from_c.d2_frame_from_ref()).norm(), OOM * h);
  }
}

TYPED_TEST(TwoJetLTests, ConversionRoundTrip) {
  TwoJetL<TypeParam> a_from_b = TestFixture::tj_helper().make_test_two_jet();
  TwoJetR<TypeParam> b_from_a = a_from_b.right_two_jet();
  TwoJetL<TypeParam> a_from_b_2 = b_from_a.left_two_jet();

  EXPECT_FALSE(b_from_a.ref_from_frame().is_approx(a_from_b.frame_from_ref()));
  EXPECT_FALSE(
      b_from_a.d_ref_from_frame().isApprox(a_from_b.d_frame_from_ref()));
  EXPECT_FALSE(
      b_from_a.d2_ref_from_frame().isApprox(a_from_b.d2_frame_from_ref()));
  EXPECT_TRUE(a_from_b.is_approx(a_from_b_2));
}

TYPED_TEST(UnframedTwoJetLTests, ConversionAssociativity) {
  TwoJetL<TypeParam> a_from_b = TestFixture::tj_helper().make_test_two_jet();
  TwoJetL<TypeParam> b_from_c = TestFixture::tj_helper().make_test_two_jet();
  TwoJetR<TypeParam> b_from_a = a_from_b.right_two_jet();
  TwoJetR<TypeParam> c_from_b = b_from_c.right_two_jet();

  TwoJetL<TypeParam> a_from_c = a_from_b * b_from_c;
  TwoJetR<TypeParam> c_from_a = c_from_b * b_from_a;

  EXPECT_TRUE(c_from_a.is_approx(a_from_c.right_two_jet()));
}

TYPED_TEST(TwoJetLTests, IsApproxTest) {
  TwoJetL<TypeParam> test_two_jet_0 =
      TestFixture::tj_helper().make_test_two_jet();
  ;
  TwoJetL<TypeParam> test_two_jet_1 =
      TestFixture::tj_helper().make_test_two_jet();
  ;
  EXPECT_FALSE(test_two_jet_0.is_approx(test_two_jet_1));
  test_two_jet_0.set_frame_from_ref(test_two_jet_1.frame_from_ref());
  EXPECT_FALSE(test_two_jet_0.is_approx(test_two_jet_1));
  test_two_jet_0.set_d_frame_from_ref(test_two_jet_1.d_frame_from_ref());
  EXPECT_FALSE(test_two_jet_0.is_approx(test_two_jet_1));
  test_two_jet_0.set_d2_frame_from_ref(test_two_jet_1.d2_frame_from_ref());
  EXPECT_TRUE(test_two_jet_0.is_approx(test_two_jet_1));
}

// TwoJetR specific tests. These tests use methods only defined in TwoJetR
// and must be tested separately.
template <typename Group>
class TwoJetRTests : public TwoJetTestsBase<TwoJetR<Group>> {
 protected:
  TwoJetR<Group> extrapolate(const TwoJetR<Group> &x, const double dt) {
    constexpr double HALF = 0.5;
    return TwoJetR<Group>{
        x.ref_from_frame() * Group::exp(
                                 dt * (x.d_ref_from_frame() +
                                       HALF * dt * x.d2_ref_from_frame())),
        x.d_ref_from_frame() + dt * x.d2_ref_from_frame(),
        x.d2_ref_from_frame()};
  }
};

TYPED_TEST_SUITE(TwoJetRTests, LieGroupTypes);

template <typename T>
class UnframedTwoJetRTests : public TwoJetRTests<T> {};

TYPED_TEST_SUITE(UnframedTwoJetRTests, UnframedTypes);

TYPED_TEST(TwoJetRTests, CompositionByInverseIsIdentityAndZeros) {
  std::vector<TwoJetR<TypeParam>> test_elements =
      TestFixture::tj_helper().make_test_two_jet_elements(NUM_TRIES);
  for (const auto &test_tj : test_elements) {
    TwoJetR<TypeParam> id_tj = TwoJetR<TypeParam>::identity();
    TwoJetR<TypeParam> expected_id_tj = test_tj.inverse() * test_tj;
    EXPECT_TRUE(
        expected_id_tj.ref_from_frame().is_approx(id_tj.ref_from_frame()));
    // TwoJetR results in inexact inversions, unlike TwoJetL
    // TODO(https://app.asana.com/0/1202178773526279/1203942988562265/f)
    constexpr double TOLERANCE = 1e-8;
    EXPECT_TRUE(expected_id_tj.d_ref_from_frame().isZero(TOLERANCE));
    EXPECT_TRUE(expected_id_tj.d2_ref_from_frame().isZero(TOLERANCE));
  }
}

TYPED_TEST(UnframedTwoJetRTests, NumericalDerivativesTest) {
  for (unsigned int i = 0; i < NUM_TRIES; ++i) {
    TwoJetR<TypeParam> a_from_b = TestFixture::tj_helper().make_test_two_jet();
    TwoJetR<TypeParam> b_from_c = TestFixture::tj_helper().make_test_two_jet();
    TwoJetR<TypeParam> a_from_c = a_from_b * b_from_c;

    constexpr double h = 1e-8;
    auto a_from_b_p = this->extrapolate(a_from_b, h);
    auto b_from_c_p = this->extrapolate(b_from_c, h);
    auto a_from_c_p = a_from_b_p * b_from_c_p;

    typename TypeParam::TangentVector expected{
        (a_from_c_p.d_ref_from_frame() - a_from_c.d_ref_from_frame()) / h};
    // Error is expected to be somewhat larger than the step h, so order of
    // magnitude is used as the threshold.
    constexpr double OOM = 10.;
    EXPECT_LT((expected - a_from_c.d2_ref_from_frame()).norm(), OOM * h);
  }
}

TYPED_TEST(TwoJetRTests, ConversionRoundTrip) {
  TwoJetR<TypeParam> a_from_b = TestFixture::tj_helper().make_test_two_jet();
  TwoJetL<TypeParam> b_from_a = a_from_b.left_two_jet();
  TwoJetR<TypeParam> a_from_b_2 = b_from_a.right_two_jet();

  EXPECT_FALSE(b_from_a.frame_from_ref().is_approx(a_from_b.ref_from_frame()));
  EXPECT_FALSE(
      b_from_a.d_frame_from_ref().isApprox(a_from_b.d_ref_from_frame()));
  EXPECT_FALSE(
      b_from_a.d2_frame_from_ref().isApprox(a_from_b.d2_ref_from_frame()));
  EXPECT_TRUE(a_from_b.is_approx(a_from_b_2));
}

TYPED_TEST(UnframedTwoJetRTests, ConversionAssociativity) {
  TwoJetR<TypeParam> a_from_b = TestFixture::tj_helper().make_test_two_jet();
  TwoJetR<TypeParam> b_from_c = TestFixture::tj_helper().make_test_two_jet();
  TwoJetL<TypeParam> b_from_a = a_from_b.left_two_jet();
  TwoJetL<TypeParam> c_from_b = b_from_c.left_two_jet();

  TwoJetR<TypeParam> a_from_c = a_from_b * b_from_c;
  TwoJetL<TypeParam> c_from_a = c_from_b * b_from_a;

  EXPECT_TRUE(c_from_a.is_approx(a_from_c.left_two_jet()));
}

TYPED_TEST(TwoJetRTests, IsApproxTest) {
  TwoJetR<TypeParam> test_two_jet_0 =
      TestFixture::tj_helper().make_test_two_jet();
  TwoJetR<TypeParam> test_two_jet_1 =
      TestFixture::tj_helper().make_test_two_jet();
  EXPECT_FALSE(test_two_jet_0.is_approx(test_two_jet_1));
  test_two_jet_0.set_ref_from_frame(test_two_jet_1.ref_from_frame());
  EXPECT_FALSE(test_two_jet_0.is_approx(test_two_jet_1));
  test_two_jet_0.set_d_ref_from_frame(test_two_jet_1.d_ref_from_frame());
  EXPECT_FALSE(test_two_jet_0.is_approx(test_two_jet_1));
  test_two_jet_0.set_d2_ref_from_frame(test_two_jet_1.d2_ref_from_frame());
  EXPECT_TRUE(test_two_jet_0.is_approx(test_two_jet_1));
}

}  // namespace resim::curves
