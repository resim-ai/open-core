// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <cstdint>
#include <random>

#include "resim/msg/header.pb.h"
#include "resim/msg/transform.pb.h"
#include "resim/testing/fuzz_helpers.hh"
#include "resim/testing/random_matrix.hh"
#include "resim/transforms/proto/se3_to_proto.hh"
#include "resim/utils/inout.hh"
#include "resim/utils/uuid.hh"

namespace resim::msg {

template <typename Rng>
Header random_element(TypeTag<Header> /*unused*/, InOut<Rng> rng) {
  Header result;

  // Need to use int32 since that's all ROS supports
  result.mutable_stamp()->set_seconds(random_element(TypeTag<int32_t>(), rng));
  constexpr int32_t NANOS_LB = 0;
  constexpr int32_t NANOS_UB = 1000000000;
  std::uniform_int_distribution<int32_t> dist{NANOS_LB, NANOS_UB};
  result.mutable_stamp()->set_nanos(dist(*rng));
  result.set_frame_id(UUID::new_uuid().to_string());
  return result;
}

template <typename Rng>
TransformStamped random_element(
    TypeTag<TransformStamped> /*unused*/,
    InOut<Rng> rng) {
  TransformStamped result;

  result.mutable_header()->CopyFrom(random_element<Header>(rng));
  result.set_child_frame_id(UUID::new_uuid().to_string());
  const transforms::SE3 transform{transforms::SE3::exp(
      testing::random_vector<transforms::SE3::TangentVector>(*rng))};
  pack(transform, result.mutable_transform());
  return result;
}

template <typename Rng>
TransformArray random_element(
    TypeTag<TransformArray> /*unused*/,
    InOut<Rng> rng) {
  TransformArray result;
  constexpr int MIN_ELEMENTS = 1;
  constexpr int MAX_ELEMENTS = 10;
  std::uniform_int_distribution<int> dist{MIN_ELEMENTS, MAX_ELEMENTS};
  const int num_elements = dist(*rng);
  for (int ii = 0; ii < num_elements; ++ii) {
    result.add_transforms()->CopyFrom(random_element<TransformStamped>(rng));
  }
  return result;
}

bool verify_equality(const Header &a, const Header &b);

bool verify_equality(const TransformStamped &a, const TransformStamped &b);

bool verify_equality(const TransformArray &a, const TransformArray &b);

}  // namespace resim::msg
