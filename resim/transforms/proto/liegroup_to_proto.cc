#include "resim/transforms/proto/liegroup_to_proto.hh"

#include "resim/assert/assert.hh"
#include "resim/math/proto/matrix_to_proto.hh"
#include "resim/transforms/proto/frame_3_to_proto.hh"
#include "resim/transforms/proto/se3.pb.h"
#include "resim/transforms/proto/so3.pb.h"
#include "resim/transforms/se3.hh"
#include "resim/transforms/so3.hh"
#include "resim/utils/inout.hh"

namespace resim::transforms::proto {

template <transforms::LieGroupType Group, typename Msg>
void pack_liegroup(const Group &in, Msg *out) {
  REASSERT(out != nullptr, "Can't pack into invalid proto!");
  out->Clear();
  math::proto::pack_matrix(in.log(), out->mutable_algebra());
  pack(in.into(), out->mutable_into());
  pack(in.from(), out->mutable_from());
}

template <transforms::LieGroupType Group, typename Msg>
void unpack_liegroup(const Msg &in, InOut<Group> out) {
  typename Group::TangentVector alg;
  math::proto::unpack_matrix(in.algebra(), InOut(alg));
  *out = Group::exp(alg, unpack(in.into()), unpack(in.from()));
}

template void pack_liegroup(const transforms::SE3 &, SE3 *);
template void pack_liegroup(const transforms::SO3 &, SO3 *);
template void unpack_liegroup(const SE3 &, InOut<transforms::SE3>);
template void unpack_liegroup(const SO3 &, InOut<transforms::SO3>);

}  // namespace resim::transforms::proto
