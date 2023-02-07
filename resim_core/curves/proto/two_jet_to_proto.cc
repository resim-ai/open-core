#include "resim_core/curves/proto/two_jet_to_proto.hh"

#include <glog/logging.h>

#include "resim_core/curves/proto/two_jet.pb.h"
#include "resim_core/curves/two_jet.hh"
#include "resim_core/math/proto/matrix_to_proto.hh"
#include "resim_core/transforms/framed_group.hh"
#include "resim_core/transforms/proto/fse3_to_proto.hh"
#include "resim_core/transforms/proto/fso3_to_proto.hh"
#include "resim_core/transforms/proto/se3_to_proto.hh"
#include "resim_core/transforms/proto/so3_to_proto.hh"
#include "resim_core/transforms/se3.hh"
#include "resim_core/transforms/so3.hh"

namespace resim::curves::proto {

template <typename Group, typename Msg>
void pack_two_jet(const TwoJetL<Group> &in, Msg *out) {
  CHECK(out != nullptr) << "Can't pack into invalid proto!";
  out->Clear();
  pack(in.frame_from_ref(), out->mutable_frame_from_ref());
  math::proto::pack_matrix(
      in.d_frame_from_ref(),
      out->mutable_d_frame_from_ref());
  math::proto::pack_matrix(
      in.d2_frame_from_ref(),
      out->mutable_d2_frame_from_ref());
}

template void pack_two_jet(const TwoJetL<transforms::SE3> &, TwoJetL_SE3 *);
template void pack_two_jet(const TwoJetL<transforms::SO3> &, TwoJetL_SO3 *);
template void pack_two_jet(const TwoJetL<transforms::FSE3> &, TwoJetL_FSE3 *);
template void pack_two_jet(const TwoJetL<transforms::FSO3> &, TwoJetL_FSO3 *);

template <typename Group, typename Msg>
void unpack_two_jet(const Msg &in, InOut<TwoJetL<Group>> out) {
  Group group = unpack(in.frame_from_ref());
  typename Group::TangentVector d_frame_from_ref;
  typename Group::TangentVector d2_frame_from_ref;
  math::proto::unpack_matrix(in.d_frame_from_ref(), InOut(d_frame_from_ref));
  math::proto::unpack_matrix(in.d2_frame_from_ref(), InOut(d2_frame_from_ref));
  *out = TwoJetL<Group>(group, d_frame_from_ref, d2_frame_from_ref);
}

template void unpack_two_jet(
    const TwoJetL_SE3 &,
    InOut<TwoJetL<transforms::SE3>>);
template void unpack_two_jet(
    const TwoJetL_SO3 &,
    InOut<TwoJetL<transforms::SO3>>);
template void unpack_two_jet(
    const TwoJetL_FSE3 &,
    InOut<TwoJetL<transforms::FSE3>>);
template void unpack_two_jet(
    const TwoJetL_FSO3 &,
    InOut<TwoJetL<transforms::FSO3>>);

}  // namespace resim::curves::proto
