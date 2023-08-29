// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <cstdint>
#include <random>

#include "resim/msg/header.pb.h"
#include "resim/msg/odometry.pb.h"
#include "resim/msg/pose.pb.h"
#include "resim/msg/transform.pb.h"
#include "resim/testing/fuzz_helpers.hh"
#include "resim/testing/random_matrix.hh"
#include "resim/transforms/frame.hh"
#include "resim/transforms/proto/fuzz_helpers.hh"
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
  result.mutable_transform()->CopyFrom(
      random_element<transforms::proto::SE3>(rng));

  // We don't use frame IDs when converting to/from ROS2.
  constexpr int DIMS = 3;
  result.mutable_transform()->mutable_into()->mutable_id()->set_data(
      transforms::Frame<DIMS>::null_frame().id().to_string());
  result.mutable_transform()->mutable_from()->mutable_id()->set_data(
      transforms::Frame<DIMS>::null_frame().id().to_string());

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

template <typename Rng>
PoseWithCovariance random_element(
    TypeTag<PoseWithCovariance> /*unused*/,
    InOut<Rng> rng) {
  PoseWithCovariance result;
  result.mutable_pose()->CopyFrom(random_element<transforms::proto::SE3>(rng));

  // We don't use frame IDs when converting to/from ROS2.
  constexpr int DIMS = 3;
  result.mutable_pose()->mutable_into()->mutable_id()->set_data(
      transforms::Frame<DIMS>::null_frame().id().to_string());
  result.mutable_pose()->mutable_from()->mutable_id()->set_data(
      transforms::Frame<DIMS>::null_frame().id().to_string());

  constexpr std::size_t N = transforms::SE3::DOF;
  for (int ii = 0; ii < N * N; ++ii) {
    result.add_covariance(random_element<double>(rng));
  }
  return result;
}

template <typename Rng>
Twist random_element(TypeTag<Twist> /*unused*/, InOut<Rng> rng) {
  Twist result;
  constexpr std::size_t N = transforms::SE3::DOF;
  for (int ii = 0; ii < N; ++ii) {
    result.add_algebra(random_element<double>(rng));
  }
  return result;
}

template <typename Rng>
TwistWithCovariance random_element(
    TypeTag<TwistWithCovariance> /*unused*/,
    InOut<Rng> rng) {
  TwistWithCovariance result;
  result.mutable_twist()->CopyFrom(random_element<Twist>(rng));
  constexpr std::size_t N = transforms::SE3::DOF;
  for (int ii = 0; ii < N * N; ++ii) {
    result.add_covariance(random_element<double>(rng));
  }
  return result;
}

template <typename Rng>
Odometry random_element(TypeTag<Odometry> /*unused*/, InOut<Rng> rng) {
  Odometry result;
  result.mutable_header()->CopyFrom(random_element<Header>(rng));
  result.set_child_frame_id(UUID::new_uuid().to_string());
  result.mutable_pose()->CopyFrom(random_element<PoseWithCovariance>(rng));
  result.mutable_twist()->CopyFrom(random_element<TwistWithCovariance>(rng));
  return result;
}

bool verify_equality(const Header &a, const Header &b);

bool verify_equality(const TransformStamped &a, const TransformStamped &b);

bool verify_equality(const TransformArray &a, const TransformArray &b);

bool verify_equality(const PoseWithCovariance &a, const PoseWithCovariance &b);

bool verify_equality(const Twist &a, const Twist &b);

bool verify_equality(
    const TwistWithCovariance &a,
    const TwistWithCovariance &b);

bool verify_equality(const Odometry &a, const Odometry &b);

}  // namespace resim::msg
