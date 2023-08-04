// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/curves/two_jet_test_helpers.hh"

#include <random>
#include <vector>

#include "resim/assert/assert.hh"
#include "resim/curves/two_jet.hh"
#include "resim/curves/two_jet_concepts.hh"
#include "resim/testing/random_matrix.hh"
#include "resim/transforms/liegroup_test_helpers.hh"
#include "resim/transforms/se3.hh"
#include "resim/transforms/so3.hh"

namespace resim::curves {

namespace {
constexpr auto LOW_COUNT =
    "The minimum number of test elements you can request is seven. Please "
    "increase the count.";

template <typename TwoJet, typename Rng>
typename TwoJet::GroupType::TangentVector test_vector(Rng &rng) {
  return testing::random_vector<typename TwoJet::GroupType::TangentVector>(rng);
}

}  // namespace

template <curves::TwoJetType TwoJet>
TwoJetTestHelper<TwoJet>::TwoJetTestHelper(const unsigned int seed) {
  rng_.seed(seed);
}

template <curves::TwoJetType TwoJet>
TwoJet TwoJetTestHelper<TwoJet>::make_test_two_jet() {
  using Group = typename TwoJet::GroupType;
  const auto point_from_ref =
      Group::exp(test_vector<TwoJet>(rng()), INTO_FRAME, FROM_FRAME);
  return TwoJet(
      point_from_ref,
      test_vector<TwoJet>(rng()),
      test_vector<TwoJet>(rng()));
}

template <curves::TwoJetType TwoJet>
std::vector<TwoJet> TwoJetTestHelper<TwoJet>::make_test_two_jet_elements(
    const unsigned count) {
  // How many elements to create
  REASSERT(count >= detail::MIN_TEST_ELEMENTS, LOW_COUNT);
  std::vector<TwoJet> two_jets;
  // Delegate the creation of edge case and random inputs to the lie group
  // helpers then construct the TwoJet objects
  const auto group_elements =
      transforms::make_test_group_elements<typename TwoJet::GroupType>(count);
  const auto algebra_elements_1 =
      transforms::make_test_algebra_elements<typename TwoJet::GroupType>(count);
  const auto algebra_elements_2 =
      transforms::make_test_algebra_elements<typename TwoJet::GroupType>(count);
  two_jets.reserve(count);
  for (int i = 0; i < count; ++i) {
    typename TwoJet::GroupType framed_group = group_elements[i];
    framed_group.set_frames(INTO_FRAME, FROM_FRAME);
    two_jets.push_back(
        TwoJet(framed_group, algebra_elements_1[i], algebra_elements_2[i]));
  }
  two_jets.resize(count);
  return two_jets;
}

template class TwoJetTestHelper<TwoJetL<transforms::SE3>>;
template class TwoJetTestHelper<TwoJetL<transforms::SO3>>;
template class TwoJetTestHelper<TwoJetR<transforms::SE3>>;
template class TwoJetTestHelper<TwoJetR<transforms::SO3>>;

}  // namespace resim::curves
