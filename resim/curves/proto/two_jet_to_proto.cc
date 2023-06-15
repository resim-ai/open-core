#include "resim/curves/proto/two_jet_to_proto.hh"

#include "resim/assert/assert.hh"
#include "resim/curves/proto/two_jet.pb.h"
#include "resim/curves/two_jet.hh"
#include "resim/math/proto/matrix_to_proto.hh"
#include "resim/transforms/proto/se3_to_proto.hh"
#include "resim/transforms/proto/so3_to_proto.hh"
#include "resim/transforms/se3.hh"
#include "resim/transforms/so3.hh"

namespace resim::curves::proto {

template <typename Group, typename Msg>
void pack_two_jetl(const TwoJetL<Group> &in, Msg *out) {
  REASSERT(out != nullptr, "Can't pack into invalid proto!");
  out->Clear();
  pack(in.frame_from_ref(), out->mutable_frame_from_ref());
  math::proto::pack_matrix(
      in.d_frame_from_ref(),
      out->mutable_d_frame_from_ref());
  math::proto::pack_matrix(
      in.d2_frame_from_ref(),
      out->mutable_d2_frame_from_ref());
}

template <typename Group, typename Msg>
void pack_two_jetr(const TwoJetR<Group> &in, Msg *out) {
  REASSERT(out != nullptr, "Can't pack into invalid proto!");
  out->Clear();
  pack(in.ref_from_frame(), out->mutable_ref_from_frame());
  math::proto::pack_matrix(
      in.d_ref_from_frame(),
      out->mutable_d_ref_from_frame());
  math::proto::pack_matrix(
      in.d2_ref_from_frame(),
      out->mutable_d2_ref_from_frame());
}

template void pack_two_jetl(const TwoJetL<transforms::SE3> &, TwoJetL_SE3 *);
template void pack_two_jetl(const TwoJetL<transforms::SO3> &, TwoJetL_SO3 *);
template void pack_two_jetr(const TwoJetR<transforms::SE3> &, TwoJetR_SE3 *);
template void pack_two_jetr(const TwoJetR<transforms::SO3> &, TwoJetR_SO3 *);

template <typename Group, typename Msg>
void unpack_two_jetl(const Msg &in, InOut<TwoJetL<Group>> out) {
  Group group = unpack(in.frame_from_ref());
  typename Group::TangentVector d_frame_from_ref;
  typename Group::TangentVector d2_frame_from_ref;
  math::proto::unpack_matrix(in.d_frame_from_ref(), InOut(d_frame_from_ref));
  math::proto::unpack_matrix(in.d2_frame_from_ref(), InOut(d2_frame_from_ref));
  *out = TwoJetL<Group>(group, d_frame_from_ref, d2_frame_from_ref);
}

template <typename Group, typename Msg>
void unpack_two_jetr(const Msg &in, InOut<TwoJetR<Group>> out) {
  Group group = unpack(in.ref_from_frame());
  typename Group::TangentVector d_ref_from_frame;
  typename Group::TangentVector d2_ref_from_frame;
  math::proto::unpack_matrix(in.d_ref_from_frame(), InOut(d_ref_from_frame));
  math::proto::unpack_matrix(in.d2_ref_from_frame(), InOut(d2_ref_from_frame));
  *out = TwoJetR<Group>(group, d_ref_from_frame, d2_ref_from_frame);
}

template void unpack_two_jetl(
    const TwoJetL_SE3 &,
    InOut<TwoJetL<transforms::SE3>>);
template void unpack_two_jetl(
    const TwoJetL_SO3 &,
    InOut<TwoJetL<transforms::SO3>>);
template void unpack_two_jetr(
    const TwoJetR_SE3 &,
    InOut<TwoJetR<transforms::SE3>>);
template void unpack_two_jetr(
    const TwoJetR_SO3 &,
    InOut<TwoJetR<transforms::SO3>>);
}  // namespace resim::curves::proto
