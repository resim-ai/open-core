// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/curves/extrapolate_two_jet.hh"

#include "resim/transforms/se3.hh"
#include "resim/transforms/so3.hh"

namespace resim::curves {

namespace {

// General implementation of the extrapolation logic which forwards the given
// arguments to the exponential in order to initialize the group's frames.
template <transforms::LieGroupType Group, typename... FrameArgs>
TwoJetL<Group> extrapolate_two_jet_impl(
    const TwoJetL<Group> &two_jet,
    const double dt,
    FrameArgs &&...frame_args) {
  TwoJetL<Group> result{two_jet};

  constexpr double ONE_HALF = 0.5;
  const Group next_from_prev_frame = Group::exp(
      dt * (two_jet.d_frame_from_ref() +
            ONE_HALF * dt * two_jet.d2_frame_from_ref()),
      frame_args...);
  result.set_frame_from_ref(next_from_prev_frame * two_jet.frame_from_ref());
  result.set_d_frame_from_ref(
      dt * two_jet.d2_frame_from_ref() + two_jet.d_frame_from_ref());

  return result;
}

// Overload of the implementation which works specifically for framed groups
// without any arguments. In this case, we can use the extrapolate_two_jet
// function and default to the pre-existing into frame.
template <transforms::LieGroupType Group>
TwoJetL<Group> extrapolate_two_jet_impl(
    const TwoJetL<Group> &two_jet,
    double dt) {
  return extrapolate_two_jet(two_jet, dt, two_jet.frame_from_ref().into());
}
}  // namespace

template <transforms::LieGroupType Group>
TwoJetL<Group> extrapolate_two_jet(
    const TwoJetL<Group> &two_jet,
    double dt,
    const transforms::Frame<Group::DIMS> &return_frame) {
  REASSERT(two_jet.frame_from_ref().is_framed());
  return extrapolate_two_jet_impl(
      two_jet,
      dt,
      return_frame,
      two_jet.frame_from_ref().into());
}

template <transforms::LieGroupType Group>
TwoJetL<Group> extrapolate_two_jet(const TwoJetL<Group> &two_jet, double dt) {
  // Default to preserving frames in the extrapolated two jet.
  return extrapolate_two_jet_impl(
      two_jet,
      dt,
      two_jet.frame_from_ref().into(),
      two_jet.frame_from_ref().into());
}

template TwoJetL<transforms::SE3> extrapolate_two_jet(
    const TwoJetL<transforms::SE3> &,
    double);
template TwoJetL<transforms::SO3> extrapolate_two_jet(
    const TwoJetL<transforms::SO3> &,
    double);
template TwoJetL<transforms::SE3> extrapolate_two_jet(
    const TwoJetL<transforms::SE3> &,
    double,
    const transforms::Frame<transforms::SE3::DIMS> &);
template TwoJetL<transforms::SO3> extrapolate_two_jet(
    const TwoJetL<transforms::SO3> &,
    double,
    const transforms::Frame<transforms::SO3::DIMS> &);
}  // namespace resim::curves
