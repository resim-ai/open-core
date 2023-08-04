// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/curves/optimization/two_jet_tangent_space.hh"

#include "resim/math/vector_partition.hh"
#include "resim/transforms/se3.hh"
#include "resim/transforms/so3.hh"

namespace resim::curves::optimization {
using transforms::SE3;
using transforms::SO3;

template <transforms::LieGroupType Group>
TwoJetL<Group> accumulate(
    const TwoJetL<Group> &two_jet,
    const TwoJetTangentVector<Group> &tangent) {
  using Part = TwoJetPartition<Group>;

  const Group &frame_from_ref{two_jet.frame_from_ref()};

  return TwoJetL<Group>{
      Group::exp(
          math::get_block<Part, FRAME_FROM_REF>(tangent),
          frame_from_ref.into(),
          frame_from_ref.into()) *
          frame_from_ref,
      two_jet.d_frame_from_ref() +
          math::get_block<Part, D_FRAME_FROM_REF>(tangent),
      two_jet.d2_frame_from_ref() +
          math::get_block<Part, D2_FRAME_FROM_REF>(tangent),
  };
}

template <transforms::LieGroupType Group>
TwoJetTangentVector<Group> difference(
    const TwoJetL<Group> &a,
    const TwoJetL<Group> &b) {
  using Part = TwoJetPartition<Group>;

  TwoJetTangentVector<Group> tangent;
  math::get_block<Part, FRAME_FROM_REF>(tangent) =
      (a.frame_from_ref() * b.frame_from_ref().inverse()).log();
  math::get_block<Part, D_FRAME_FROM_REF>(tangent) =
      a.d_frame_from_ref() - b.d_frame_from_ref();
  math::get_block<Part, D2_FRAME_FROM_REF>(tangent) =
      a.d2_frame_from_ref() - b.d2_frame_from_ref();

  return tangent;
}

template TwoJetL<SE3> accumulate<SE3>(
    const TwoJetL<SE3> &two_jet,
    const TwoJetTangentVector<SE3> &tangent);

template TwoJetL<SO3> accumulate<SO3>(
    const TwoJetL<SO3> &two_jet,
    const TwoJetTangentVector<SO3> &tangent);

template TwoJetTangentVector<SE3> difference<SE3>(
    const TwoJetL<SE3> &a,
    const TwoJetL<SE3> &b);

template TwoJetTangentVector<SO3> difference<SO3>(
    const TwoJetL<SO3> &a,
    const TwoJetL<SO3> &b);

}  // namespace resim::curves::optimization
