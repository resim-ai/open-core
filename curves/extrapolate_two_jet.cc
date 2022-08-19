
#include "curves/extrapolate_two_jet.hh"

#include "transforms/framed_group_concept.hh"
#include "transforms/se3.hh"
#include "transforms/so3.hh"

namespace resim::curves {

namespace {

// General implementation of the extrapolation logic which forwards the given
// arguments to the exponential in order to initialize the group's frames if
// it's a framed group.
template <transforms::LieGroupType Group, typename... FrameArgs>
TwoJet<Group> extrapolate_two_jet_impl(
    const TwoJet<Group> &two_jet,
    const double dt,
    FrameArgs &&...frame_args) {
  TwoJet<Group> result{two_jet};

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
// without any arguments. In this case, we can use the extrapolate two jet
// function and default to the pre-existing into frame.
template <transforms::LieGroupType Group>
TwoJet<transforms::FramedGroup<Group>> extrapolate_two_jet_impl(
    const TwoJet<transforms::FramedGroup<Group>> &two_jet,
    double dt) {
  return extrapolate_two_jet(two_jet, dt, two_jet.frame_from_ref().into());
}
}  // namespace

template <transforms::LieGroupType Group>
TwoJet<transforms::FramedGroup<Group>> extrapolate_two_jet(
    const TwoJet<transforms::FramedGroup<Group>> &two_jet,
    double dt,
    const transforms::Frame<Group::DIMS> &return_frame) {
  return extrapolate_two_jet_impl(
      two_jet,
      dt,
      return_frame,
      two_jet.frame_from_ref().into());
}

template <transforms::LieGroupType Group>
TwoJet<Group> extrapolate_two_jet(const TwoJet<Group> &two_jet, double dt) {
  return extrapolate_two_jet_impl(two_jet, dt);
}

template TwoJet<transforms::SE3> extrapolate_two_jet(
    const TwoJet<transforms::SE3> &,
    double);
template TwoJet<transforms::SO3> extrapolate_two_jet(
    const TwoJet<transforms::SO3> &,
    double);
template TwoJet<transforms::FSE3> extrapolate_two_jet(
    const TwoJet<transforms::FSE3> &,
    double);
template TwoJet<transforms::FSO3> extrapolate_two_jet(
    const TwoJet<transforms::FSO3> &,
    double);

}  // namespace resim::curves
