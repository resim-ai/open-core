#pragma once

#include "curves/two_jet.hh"
#include "transforms/frame.hh"
#include "transforms/framed_group.hh"
#include "transforms/liegroup_concepts.hh"

namespace resim::curves {

// This function template takes a two jet and extrapolates it forward by time dt
// assuming constant acceleration. For framed group, the default behavior is to
// preserve the from and into coordinate frames from the input TwoJet. If a user
// wants to specify a new frame for the lefthand side of the two-jet, they
// should use the overload below.
// @param[in] two_jet - The two jet to extrapolate.
// @param[in] dt - The time to extrapolate forward.
template <transforms::LieGroupType Group>
TwoJet<Group> extrapolate_two_jet(const TwoJet<Group> &two_jet, double dt);

// An overload of the above specifically for framed groups that lets you specify
// what into frame you want the resulting TwoJet to have.
// @param[in] two_jet - The two jet to extrapolate.
// @param[in] dt - The time to extrapolate forward.
// @param[in] return_frame - The into frame we want the resulting two jet to
//                           have.
template <transforms::LieGroupType Group>
TwoJet<transforms::FramedGroup<Group>> extrapolate_two_jet(
    const TwoJet<transforms::FramedGroup<Group>> &two_jet,
    double dt,
    const transforms::Frame<Group::DIMS> &return_frame);

}  // namespace resim::curves
