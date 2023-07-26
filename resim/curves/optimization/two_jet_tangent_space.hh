//
// two_jet_tangent_space.hh
//
// This library contains the definition of a tangent vector space to the
// composite manifold g x Tg x Tg for a Lie Group g. In other words it
// describes how a vector with 3 * D entries describes the derivative of a
// TwoJet on a group g. We specifically do this for TwoJetL (rather than
// TwoJetR) and use the left tangent space of the group g because this ends up
// being useful for optimizing TwoJets and TCurves to match a set of
// observations.
//
// The TwoJetTangentVector<Group> alias template is an Eigen vector type
// consisting in order of the group left tangent, first derivative tangent (just
// in R^D), and second derivative tangent (also just in R^D)
//
// For example, for SE3, the first six entries of the TwoJetTangentVector<SE3>
// describe the left tangent of the SE3, the next six describe the tangent of
// d_frame_from_ref, and the final six the tangent of d2_frame_from ref. The
// whole vector has length 18.
//

#include "resim/curves/two_jet.hh"
#include "resim/math/vector_partition.hh"
#include "resim/transforms/liegroup_concepts.hh"

#pragma once

namespace resim::curves::optimization {

//
// Code to describe the tangent vector partition for the left two jet:
//

enum TwoJetBlockIndices {
  FRAME_FROM_REF = 0,
  D_FRAME_FROM_REF = 1,
  D2_FRAME_FROM_REF = 2,
};

template <transforms::LieGroupType Group>
using TwoJetPartition =
    math::VectorPartition<Group::DOF, Group::DOF, Group::DOF>;

template <transforms::LieGroupType Group>
static constexpr int TWO_JET_DOF =
    math::VectorPartitionDim<TwoJetPartition<Group>>::value;

template <transforms::LieGroupType Group>
using TwoJetTangentVector = Eigen::Matrix<double, TWO_JET_DOF<Group>, 1>;

template <transforms::LieGroupType Group>
using TwoJetTangentMapping =
    Eigen::Matrix<double, TWO_JET_DOF<Group>, TWO_JET_DOF<Group>>;

// Add a tangent vector onto the two jet using the exponential map for the Lie
// Group part, and standard addition for the vector parts. This is the left
// plus operator of the composite manifold as described here:
// https://arxiv.org/pdf/1812.01537.pdf
// @param[in] two_jet - The two jet to add to.
// @param[in] tangent - The tangent to add.
// @returns The sum of the two_jet and tangent as described above.
template <transforms::LieGroupType Group>
TwoJetL<Group> accumulate(
    const TwoJetL<Group> &two_jet,
    const TwoJetTangentVector<Group> &tangent);

// Find a tangent vector v such that accumulate(b, v) == a for accumulate() as
// defined above. This is the left minus operator of the composite manifold as
// described here: https://arxiv.org/pdf/1812.01537.pdf
// @param[in] a - The minuend to subtract from.
// @param[in] b - The subtrahend to subtract.
// @returns The tangent vector such that accumulate(b, v) == a.
template <transforms::LieGroupType Group>
TwoJetTangentVector<Group> difference(
    const TwoJetL<Group> &a,
    const TwoJetL<Group> &b);

}  // namespace resim::curves::optimization
