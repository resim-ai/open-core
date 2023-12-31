// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <random>
#include <vector>

#include "resim/curves/two_jet_concepts.hh"
#include "resim/transforms/frame.hh"

namespace resim::curves {

namespace detail {
// The minimum number of TwoJet test elements to request
constexpr unsigned MIN_TEST_ELEMENTS = 7;
}  // namespace detail

template <curves::TwoJetType TwoJet>
class TwoJetTestHelper {
 protected:
  std::mt19937 &rng() { return rng_; }

 private:
  std::mt19937 rng_;

 public:
  using Frame = transforms::Frame<TwoJet::GroupType::DIMS>;
  inline static const Frame INTO_FRAME = Frame::new_frame();
  inline static const Frame FROM_FRAME = Frame::new_frame();
  TwoJetTestHelper() = default;

  // Construct a helper class that can spin up random TwoJets
  explicit TwoJetTestHelper(unsigned int seed);

  // Builds and returns a single TwoJet with randomly populated elements
  TwoJet make_test_two_jet();

  // Builds and returns a vector containing a fixed number of TwoJet objects
  // that are useful for testing. The TwoJets are randomly populated elements
  // @param[in] count - Optionally, the number of TwoJets to return. The default
  //                    and the minimum are both seven. If you request less the
  //                    function will check-fail.
  std::vector<TwoJet> make_test_two_jet_elements(
      unsigned count = detail::MIN_TEST_ELEMENTS);
};

}  // namespace resim::curves
