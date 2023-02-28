#include "resim_core/transforms/proto/liegroup_to_proto.hh"

#include "resim_core/assert/assert.hh"
#include "resim_core/math/proto/matrix_to_proto.hh"
#include "resim_core/transforms/framed_group.hh"
#include "resim_core/transforms/proto/se3.pb.h"
#include "resim_core/transforms/proto/so3.pb.h"
#include "resim_core/transforms/se3.hh"
#include "resim_core/transforms/so3.hh"
#include "resim_core/utils/inout.hh"

namespace resim::transforms::proto {

template <transforms::LieGroupType Group, typename Msg>
void pack_liegroup(const Group &in, Msg *out) {
  REASSERT(out != nullptr, "Can't pack into invalid proto!");
  out->Clear();
  math::proto::pack_matrix(in.log(), out->mutable_algebra());
}

template <transforms::LieGroupType Group, typename Msg>
void unpack_liegroup(const Msg &in, InOut<Group> out) {
  typename Group::TangentVector alg;
  math::proto::unpack_matrix(in.algebra(), InOut(alg));
  *out = Group::exp(alg);
}

template void pack_liegroup(const transforms::SE3 &, SE3 *);
template void pack_liegroup(const transforms::SO3 &, SO3 *);
template void pack_liegroup(const transforms::FSE3 &, SE3 *);
template void pack_liegroup(const transforms::FSO3 &, SO3 *);
template void unpack_liegroup(const SE3 &, InOut<transforms::SE3>);
template void unpack_liegroup(const SO3 &, InOut<transforms::SO3>);

}  // namespace resim::transforms::proto
