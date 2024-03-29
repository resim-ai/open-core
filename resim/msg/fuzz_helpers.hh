// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <Eigen/Dense>
#include <array>
#include <cstdint>
#include <limits>
#include <random>

#include "resim/geometry/proto/fuzz_helpers.hh"
#include "resim/msg/byte_swap_helpers.hh"
#include "resim/msg/detection.pb.h"
#include "resim/msg/header.pb.h"
#include "resim/msg/navsat.pb.h"
#include "resim/msg/odometry.pb.h"
#include "resim/msg/pose.pb.h"
#include "resim/msg/primitives.pb.h"
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
  result.mutable_stamp()->CopyFrom(
      resim::random_element<google::protobuf::Timestamp>(rng));
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
  constexpr int MIN_ELEMENTS = 5;
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

template <typename Rng>
ObjectHypothesis random_element(
    TypeTag<ObjectHypothesis> /*unused*/,
    InOut<Rng> rng) {
  ObjectHypothesis result;
  result.set_class_id(UUID::new_uuid().to_string());
  constexpr double MIN = 0.;
  constexpr double MAX = 1.;
  std::uniform_real_distribution<double> dist{MIN, MAX};
  result.set_score(dist(*rng));
  return result;
}

template <typename Rng>
ObjectHypothesisWithPose random_element(
    TypeTag<ObjectHypothesisWithPose> /*unused*/,
    InOut<Rng> rng) {
  ObjectHypothesisWithPose result;
  result.mutable_hypothesis()->CopyFrom(random_element<ObjectHypothesis>(rng));
  result.mutable_pose()->CopyFrom(random_element<PoseWithCovariance>(rng));
  return result;
}

template <typename Rng>
Detection3D random_element(TypeTag<Detection3D> /*unused*/, InOut<Rng> rng) {
  Detection3D result;
  result.mutable_header()->CopyFrom(random_element<Header>(rng));

  auto bbox = random_element<geometry::proto::OrientedBoxSE3>(rng);

  // We don't use these when converting to/from ROS2
  constexpr int DIMS = 3;
  bbox.mutable_reference_from_box()->mutable_into()->mutable_id()->set_data(
      transforms::Frame<DIMS>::null_frame().id().to_string());
  bbox.mutable_reference_from_box()->mutable_from()->mutable_id()->set_data(
      transforms::Frame<DIMS>::null_frame().id().to_string());

  result.mutable_bbox()->CopyFrom(bbox);
  result.set_id(UUID::new_uuid().to_string());
  return result;
}

template <typename Rng>
BoundingBox2D random_element(
    TypeTag<BoundingBox2D> /*unused*/,
    InOut<Rng> rng) {
  BoundingBox2D result;
  result.set_center_x(random_element<double>(rng));
  result.set_center_y(random_element<double>(rng));
  result.set_theta_rad(random_element<double>(rng));
  result.set_size_x(random_element<double>(rng));
  result.set_size_y(random_element<double>(rng));
  return result;
}

template <typename Rng>
Detection2D random_element(TypeTag<Detection2D> /*unused*/, InOut<Rng> rng) {
  Detection2D result;
  result.mutable_header()->CopyFrom(random_element<Header>(rng));
  constexpr int MIN_ELEMENTS = 1;
  constexpr int MAX_ELEMENTS = 10;
  std::uniform_int_distribution<int> dist{MIN_ELEMENTS, MAX_ELEMENTS};
  const int num_elements = dist(*rng);
  for (int ii = 0; ii < num_elements; ++ii) {
    result.add_results()->CopyFrom(
        random_element<ObjectHypothesisWithPose>(rng));
  }

  result.mutable_bbox()->CopyFrom(random_element<BoundingBox2D>(rng));
  result.set_id(UUID::new_uuid().to_string());
  return result;
}

template <typename Rng>
Detection3DArray random_element(
    TypeTag<Detection3DArray> /*unused*/,
    InOut<Rng> rng) {
  Detection3DArray result;
  result.mutable_header()->CopyFrom(random_element<Header>(rng));

  constexpr int MIN_ELEMENTS = 5;
  constexpr int MAX_ELEMENTS = 10;
  std::uniform_int_distribution<int> dist{MIN_ELEMENTS, MAX_ELEMENTS};
  const int num_elements = dist(*rng);
  for (int ii = 0; ii < num_elements; ++ii) {
    result.add_detections()->CopyFrom(random_element<Detection3D>(rng));
  }

  return result;
}

template <typename Rng>
Detection2DArray random_element(
    TypeTag<Detection2DArray> /*unused*/,
    InOut<Rng> rng) {
  Detection2DArray result;
  result.mutable_header()->CopyFrom(random_element<Header>(rng));

  constexpr int MIN_ELEMENTS = 5;
  constexpr int MAX_ELEMENTS = 10;
  std::uniform_int_distribution<int> dist{MIN_ELEMENTS, MAX_ELEMENTS};
  const int num_elements = dist(*rng);
  for (int ii = 0; ii < num_elements; ++ii) {
    result.add_detections()->CopyFrom(random_element<Detection2D>(rng));
  }

  return result;
}

template <typename Rng>
NavSatFix random_element(TypeTag<NavSatFix> /*unused*/, InOut<Rng> rng) {
  constexpr std::array STATUSES = {
      NavSatFix::STATUS_NO_FIX,
      NavSatFix::STATUS_FIX,
      NavSatFix::STATUS_SBAS_FIX,
      NavSatFix::STATUS_GBAS_FIX,
  };
  constexpr std::array COVARIANCES = {
      NavSatFix::COVARIANCE_TYPE_UNKNOWN,
      NavSatFix::COVARIANCE_TYPE_APPROXIMATED,
      NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN,
      NavSatFix::COVARIANCE_TYPE_KNOWN,
  };

  NavSatFix result;
  result.mutable_header()->CopyFrom(random_element<Header>(rng));
  result.set_status(
      STATUSES.at(random_element<std::size_t>(rng) % STATUSES.size()));
  result.set_latitude_deg(random_element<double>(rng));
  result.set_longitude_deg(random_element<double>(rng));
  result.set_altitude_m(random_element<double>(rng));

  // Make a valid covariance:
  Eigen::Matrix3d covariance{testing::random_matrix<Eigen::Matrix3d>(*rng)};
  covariance = covariance.transpose() * covariance;

  for (int ii = 0; ii < covariance.rows(); ++ii) {
    for (int jj = 0; jj < covariance.cols(); ++jj) {
      result.add_position_covariance_m2(covariance(ii, jj));
    }
  }
  result.set_position_covariance_type(
      COVARIANCES.at(random_element<std::size_t>(rng) % COVARIANCES.size()));
  return result;
}

template <typename Rng>
Bool random_element(TypeTag<Bool> /*unused*/, InOut<Rng> rng) {
  std::uniform_int_distribution dist{0, 1};
  Bool result;
  result.set_data((dist(*rng) % 2) == 0);
  return result;
}

template <typename Rng>
Byte random_element(TypeTag<Byte> /*unused*/, InOut<Rng> rng) {
  Byte result;
  result.set_data(std::string{random_element<char>(rng)});
  return result;
}

template <typename Rng>
Char random_element(TypeTag<Char> /*unused*/, InOut<Rng> rng) {
  Char result;
  result.set_data(std::string{random_element<char>(rng)});
  return result;
}

template <typename Rng>
Empty random_element(TypeTag<Empty> /*unused*/, InOut<Rng> rng) {
  Empty result;
  return result;
}

template <typename Rng>
Float32 random_element(TypeTag<Float32> /*unused*/, InOut<Rng> rng) {
  Float32 result;
  result.set_data(random_element<float>(rng));
  return result;
}

template <typename Rng>
Float64 random_element(TypeTag<Float64> /*unused*/, InOut<Rng> rng) {
  Float64 result;
  result.set_data(random_element<double>(rng));
  return result;
}

template <typename Rng>
Int16 random_element(TypeTag<Int16> /*unused*/, InOut<Rng> rng) {
  Int16 result;
  set_data(random_element<int16_t>(rng), InOut{result});
  return result;
}

template <typename Rng>
Int32 random_element(TypeTag<Int32> /*unused*/, InOut<Rng> rng) {
  Int32 result;
  result.set_data(random_element<int32_t>(rng));
  return result;
}

template <typename Rng>
Int64 random_element(TypeTag<Int64> /*unused*/, InOut<Rng> rng) {
  Int64 result;
  result.set_data(random_element<int64_t>(rng));
  return result;
}

template <typename Rng>
Int8 random_element(TypeTag<Int8> /*unused*/, InOut<Rng> rng) {
  Int8 result;
  result.set_data(std::string{random_element<int8_t>(rng)});
  return result;
}

template <typename Rng>
String random_element(TypeTag<String> /*unused*/, InOut<Rng> rng) {
  String result;
  result.set_data(UUID::new_uuid().to_string());
  return result;
}

template <typename Rng>
UInt16 random_element(TypeTag<UInt16> /*unused*/, InOut<Rng> rng) {
  UInt16 result;
  set_data(random_element<int16_t>(rng), InOut{result});
  return result;
}

template <typename Rng>
UInt32 random_element(TypeTag<UInt32> /*unused*/, InOut<Rng> rng) {
  UInt32 result;
  result.set_data(random_element<uint32_t>(rng));
  return result;
}

template <typename Rng>
UInt64 random_element(TypeTag<UInt64> /*unused*/, InOut<Rng> rng) {
  UInt64 result;
  result.set_data(random_element<uint64_t>(rng));
  return result;
}

template <typename Rng>
UInt8 random_element(TypeTag<UInt8> /*unused*/, InOut<Rng> rng) {
  UInt8 result;
  result.set_data(std::string{random_element<int8_t>(rng)});
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

bool verify_equality(const ObjectHypothesis &a, const ObjectHypothesis &b);

bool verify_equality(
    const ObjectHypothesisWithPose &a,
    const ObjectHypothesisWithPose &b);

bool verify_equality(const Detection3D &a, const Detection3D &b);

bool verify_equality(const BoundingBox2D &a, const BoundingBox2D &b);

bool verify_equality(const Detection2D &a, const Detection2D &b);

bool verify_equality(const Detection3DArray &a, const Detection3DArray &b);

bool verify_equality(const Detection2DArray &a, const Detection2DArray &b);

bool verify_equality(const NavSatFix &a, const NavSatFix &b);

bool verify_equality(const Bool &a, const Bool &b);

bool verify_equality(const Byte &a, const Byte &b);

bool verify_equality(const Char &a, const Char &b);

bool verify_equality(const Empty &a, const Empty &b);

bool verify_equality(const Float32 &a, const Float32 &b);

bool verify_equality(const Float64 &a, const Float64 &b);

bool verify_equality(const Int16 &a, const Int16 &b);

bool verify_equality(const Int32 &a, const Int32 &b);

bool verify_equality(const Int64 &a, const Int64 &b);

bool verify_equality(const Int8 &a, const Int8 &b);

bool verify_equality(const String &a, const String &b);

bool verify_equality(const UInt16 &a, const UInt16 &b);

bool verify_equality(const UInt32 &a, const UInt32 &b);

bool verify_equality(const UInt64 &a, const UInt64 &b);

bool verify_equality(const UInt8 &a, const UInt8 &b);

}  // namespace resim::msg
