#include "resim/transforms/proto/framed_vector_3_to_proto.hh"

#include <Eigen/Dense>

#include "resim/assert/assert.hh"
#include "resim/math/proto/matrix_to_proto.hh"
#include "resim/transforms/proto/frame_3_to_proto.hh"
#include "resim/transforms/proto/framed_vector_3.pb.h"

namespace resim::transforms::proto {

void pack(const transforms::FramedVector<3> &in, FramedVector_3 *out) {
  REASSERT(out != nullptr, "Can't pack into invalid proto!");
  out->Clear();
  math::proto::pack_matrix(in.vector(), out->mutable_algebra());
  pack(in.frame(), out->mutable_frame());
}

transforms::FramedVector<3> unpack(const FramedVector_3 &in) {
  // Unpack the vector
  Eigen::Matrix<double, 3, 1> vector;
  math::proto::unpack_matrix(in.algebra(), InOut(vector));

  // Unpack the frame
  Frame<3> frame = unpack(in.frame());

  // Reconstruct the framed vector
  transforms::FramedVector<3> framed_vector(vector, frame);
  return framed_vector;
}

}  // namespace resim::transforms::proto
