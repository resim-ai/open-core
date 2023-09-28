// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include "resim/testing/fuzz_helpers.hh"
#include "resim/testing/random_matrix.hh"
#include "resim/transforms/proto/se3.pb.h"
#include "resim/transforms/proto/se3_to_proto.hh"
#include "resim/transforms/se3.hh"
#include "resim/utils/inout.hh"

namespace resim::transforms::proto {

template <typename Rng>
SE3 random_element(TypeTag<SE3> /*unused*/, InOut<Rng> rng) {
  SE3 result;
  const transforms::SE3 pose{transforms::SE3::exp(
      testing::random_vector<transforms::SE3::TangentVector>(*rng),
      Frame<transforms::SE3::DIMS>::new_frame(),
      Frame<transforms::SE3::DIMS>::new_frame())};
  pack(pose, &result);
  return result;
}

bool verify_equality(const SE3 &a, const SE3 &b);

}  // namespace resim::transforms::proto
