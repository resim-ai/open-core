#pragma once

#include <vector>

#include "resim_core/curves/t_curve.hh"
#include "resim_core/curves/two_jet.hh"
#include "resim_core/curves/two_jet_test_helpers.hh"
#include "resim_core/transforms/frame.hh"
#include "resim_core/transforms/liegroup_concepts.hh"

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
  TCurve<Group> make_t_curve(const std::vector<double> &times);

 private:
  TwoJetTestHelper<TwoJetL<Group>> two_jet_helper_;
};

}  // namespace resim::curves
