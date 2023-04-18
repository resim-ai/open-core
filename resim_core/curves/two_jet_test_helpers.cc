#include "resim_core/curves/two_jet_test_helpers.hh"

#include <random>
#include <vector>

#include "resim_core/assert/assert.hh"
#include "resim_core/curves/two_jet.hh"
#include "resim_core/curves/two_jet_concepts.hh"
#include "resim_core/testing/random_matrix.hh"
#include "resim_core/transforms/framed_group.hh"
#include "resim_core/transforms/framed_group_concept.hh"
#include "resim_core/transforms/liegroup_test_helpers.hh"
#include "resim_core/transforms/se3.hh"
#include "resim_core/transforms/so3.hh"

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
  auto point_from_ref = Group::exp(test_vector<TwoJet>(rng()));
  if constexpr (transforms::FramedGroupType<typename TwoJet::GroupType>) {
    point_from_ref.set_into(INTO_FRAME);
    point_from_ref.set_from(FROM_FRAME);
  }
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
    two_jets.push_back(TwoJet(
        group_elements[i],
        algebra_elements_1[i],
        algebra_elements_2[i]));
  }
  two_jets.resize(count);
  return two_jets;
}

template class TwoJetTestHelper<TwoJetL<transforms::SE3>>;
template class TwoJetTestHelper<TwoJetL<transforms::SO3>>;
template class TwoJetTestHelper<TwoJetL<transforms::FSE3>>;
template class TwoJetTestHelper<TwoJetL<transforms::FSO3>>;
template class TwoJetTestHelper<TwoJetR<transforms::SE3>>;
template class TwoJetTestHelper<TwoJetR<transforms::SO3>>;
template class TwoJetTestHelper<TwoJetR<transforms::FSE3>>;
template class TwoJetTestHelper<TwoJetR<transforms::FSO3>>;

}  // namespace resim::curves
