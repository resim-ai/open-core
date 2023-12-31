// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/curves/t_curve_test_helpers.hh"

#include <glog/logging.h>

#include <vector>

#include "resim/curves/t_curve.hh"
#include "resim/curves/two_jet.hh"
#include "resim/curves/two_jet_test_helpers.hh"
#include "resim/transforms/liegroup_concepts.hh"
#include "resim/transforms/se3.hh"
#include "resim/transforms/so3.hh"

namespace resim::curves {

template <transforms::LieGroupType Group>
TCurveTestHelper<Group>::TCurveTestHelper(const unsigned int seed)
    : two_jet_helper_(TwoJetTestHelper<TwoJetL<Group>>(seed)) {}

template <transforms::LieGroupType Group>
TwoJetTestHelper<TwoJetL<Group>> &TCurveTestHelper<Group>::two_jet_helper() {
  return two_jet_helper_;
}

template <transforms::LieGroupType Group>
TCurve<Group> TCurveTestHelper<Group>::make_t_curve(
    const std::vector<double> &times,
    bool framed) {
  TCurve<Group> test_curve;
  for (const double &t : times) {
    TwoJetL<Group> test_tj = two_jet_helper_.make_test_two_jet();
    Group group = test_tj.frame_from_ref();
    if (framed) {
      group.set_frames(POINT_FRAME, REF_FRAME);
    } else {
      group.set_unframed();
    }
    test_tj.set_frame_from_ref(group);
    test_curve.append({t, test_tj});
  }
  return test_curve;
}

template class TCurveTestHelper<transforms::SE3>;
template class TCurveTestHelper<transforms::SO3>;

}  // namespace resim::curves
