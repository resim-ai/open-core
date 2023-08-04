// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <vector>

#include "resim/curves/t_curve.hh"
#include "resim/curves/two_jet.hh"
#include "resim/curves/two_jet_test_helpers.hh"
#include "resim/transforms/frame.hh"
#include "resim/transforms/liegroup_concepts.hh"

namespace resim::curves {

template <transforms::LieGroupType Group>
class TCurveTestHelper {
 public:
  using Frame = transforms::Frame<Group::DIMS>;
  inline static const Frame REF_FRAME = Frame::new_frame();
  inline static const Frame POINT_FRAME = Frame::new_frame();
  TCurveTestHelper() = default;

  // Construct a helper class that can spin up random TCurves
  explicit TCurveTestHelper(unsigned int seed);

  // Access a reference to the underling TwoJetTestHelper
  TwoJetTestHelper<TwoJetL<Group>> &two_jet_helper();

  // Builds and returns a single TCurve with a given set of times and
  // randomly generated points, using POINT_FROM_REF framing.
  TCurve<Group> make_t_curve(
      const std::vector<double> &times,
      bool framed = true);

 private:
  TwoJetTestHelper<TwoJetL<Group>> two_jet_helper_;
};

}  // namespace resim::curves
