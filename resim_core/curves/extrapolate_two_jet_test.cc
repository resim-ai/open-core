#include "resim_core/curves/extrapolate_two_jet.hh"

#include <gtest/gtest.h>

#include <random>

#include "resim_core/common/random_vector.hh"
#include "resim_core/curves/two_jet.hh"
#include "resim_core/transforms/framed_group.hh"
#include "resim_core/transforms/framed_group_concept.hh"
#include "resim_core/transforms/liegroup_concepts.hh"
#include "resim_core/transforms/se3.hh"
#include "resim_core/transforms/so3.hh"

namespace resim::curves {

namespace {

template <transforms::LieGroupType Group, typename Rng>
Group random_group_member(Rng &&rng) {
  return Group::exp(testing::random_vector<typename Group::TangentVector>(
      std::forward<Rng>(rng)));
}

template <transforms::LieGroupType Group, typename Rng>
curves::TwoJet<Group> random_two_jet(Rng &&rng) {
  return curves::TwoJet<Group>{
      random_group_member<Group>(std::forward<Rng>(rng)),
      testing::random_vector<typename Group::TangentVector>(
          std::forward<Rng>(rng)),
      testing::random_vector<typename Group::TangentVector>(
          std::forward<Rng>(rng))};
}

}  // namespace

template <transforms::LieGroupType T>
class ExtrapolateTwoJetTests : public ::testing::Test {
 protected:
  void test_extrapolation(const double dt) {
    // SETUP
    TwoJet<T> two_jet{random_two_jet<T>(this->rng_)};

    // ACTION
    TwoJet<T> extrapolated_two_jet{extrapolate_two_jet(two_jet, dt)};

    // VERIFICATION
    EXPECT_TRUE(((extrapolated_two_jet.frame_from_ref() *
                  two_jet.frame_from_ref().inverse())
                     .log() -
                 dt * (two_jet.d_frame_from_ref() +
                       0.5 * dt * two_jet.d2_frame_from_ref()))
                    .isZero());  // isApprox() has trouble near zero
    EXPECT_TRUE(
        (extrapolated_two_jet.d_frame_from_ref() - two_jet.d_frame_from_ref())
            .isApprox(dt * two_jet.d2_frame_from_ref()));
    EXPECT_EQ(
        extrapolated_two_jet.d2_frame_from_ref(),
        two_jet.d2_frame_from_ref());

    // For framed groups, verify that the frame assignment is done
    // correctly.
    if constexpr (transforms::FramedGroupType<T>) {
      EXPECT_EQ(
          extrapolated_two_jet.frame_from_ref().into(),
          two_jet.frame_from_ref().into());
      EXPECT_EQ(
          extrapolated_two_jet.frame_from_ref().from(),
          two_jet.frame_from_ref().from());
    }
  }

  void test_frame_overload(const double dt) {
    if constexpr (transforms::FramedGroupType<T>) {
      // SETUP
      const transforms::Frame<T::DIMS> frame;
      const TwoJet<T> two_jet{random_two_jet<T>(this->rng_)};

      // ACTION
      TwoJet<T> extrapolated_two_jet_explicit{
          extrapolate_two_jet(two_jet, dt, frame)};

      // VERIFICATION
      // Verify using the other overload tested above
      TwoJet<T> extrapolated_two_jet{extrapolate_two_jet(two_jet, dt)};
      EXPECT_TRUE(
          extrapolated_two_jet.is_approx(extrapolated_two_jet_explicit));

      EXPECT_EQ(extrapolated_two_jet_explicit.frame_from_ref().into(), frame);
      EXPECT_EQ(
          extrapolated_two_jet_explicit.frame_from_ref().from(),
          two_jet.frame_from_ref().from());
    }
  }

 private:
  static constexpr unsigned SEED = 839U;
  std::mt19937 rng_{SEED};
};

using LieGroupTypes = ::testing::
    Types<transforms::SE3, transforms::SO3, transforms::FSE3, transforms::FSO3>;
TYPED_TEST_SUITE(ExtrapolateTwoJetTests, LieGroupTypes);

TYPED_TEST(ExtrapolateTwoJetTests, TestExtrapolation) {
  constexpr int NUM_TESTS = 1000;
  for (const double dt : {-0.1, 0.0, 0.1, 0.2, 0.3}) {
    for (int ii = 0; ii < NUM_TESTS; ++ii) {
      this->test_extrapolation(dt);
    }
  }
}

TYPED_TEST(ExtrapolateTwoJetTests, TestFrameOverload) {
  constexpr int NUM_TESTS = 1000;
  for (const double dt : {-0.1, 0.0, 0.1, 0.2, 0.3}) {
    for (int ii = 0; ii < NUM_TESTS; ++ii) {
      this->test_frame_overload(dt);
    }
  }
}

}  // namespace resim::curves
