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

#include "resim/converter/fuzz_helpers.hh"
#include "resim/converter/parser.hh"
#include "resim/geometry/proto/fuzz_helpers.hh"
#include "resim/msg/byte_swap_helpers.hh"
#include "resim/msg/detection.pb.h"
#include "resim/msg/header.pb.h"
#include "resim/msg/navsat.pb.h"
#include "resim/msg/odometry.pb.h"
#include "resim/msg/pose.pb.h"
#include "resim/msg/primitives.pb.h"
#include "resim/msg/transform.pb.h"
#include "resim/testing/random_matrix.hh"
#include "resim/transforms/frame.hh"
#include "resim/transforms/proto/fuzz_helpers.hh"
#include "resim/transforms/proto/se3_to_proto.hh"
#include "resim/utils/inout.hh"
#include "resim/utils/uuid.hh"

namespace resim::msg {

DEFINE_GET_PARSER(Header, PROTO_GETTER(stamp), PROTO_GETTER(frame_id));
DEFINE_GET_PARSER(TransformArray, PROTO_GETTER(transforms));
DEFINE_GET_PARSER(
    Odometry,
    PROTO_GETTER(header),
    PROTO_GETTER(child_frame_id),
    PROTO_GETTER(pose),
    PROTO_GETTER(twist));

DEFINE_GET_PARSER(
    TransformStamped,
    PROTO_GETTER(header),
    PROTO_GETTER(child_frame_id),
    PROTO_GETTER(transform));

DEFINE_GET_PARSER(
    ObjectHypothesis,
    PROTO_GETTER(class_id),
    PROTO_GETTER(score));

DEFINE_GET_PARSER(
    ObjectHypothesisWithPose,
    PROTO_GETTER(hypothesis),
    PROTO_GETTER(pose));

DEFINE_GET_PARSER(
    Detection3D,
    PROTO_GETTER(header),
    PROTO_GETTER(bbox),
    PROTO_GETTER(id));

DEFINE_GET_PARSER(
    BoundingBox2D,
    PROTO_GETTER(center_x),
    PROTO_GETTER(center_y),
    PROTO_GETTER(theta_rad),
    PROTO_GETTER(size_x),
    PROTO_GETTER(size_y));

DEFINE_GET_PARSER(
    Detection2D,
    PROTO_GETTER(header),
    PROTO_GETTER(results),
    PROTO_GETTER(bbox),
    PROTO_GETTER(id));

DEFINE_GET_PARSER(
    Detection3DArray,
    PROTO_GETTER(header),
    PROTO_GETTER(detections));

DEFINE_GET_PARSER(
    Detection2DArray,
    PROTO_GETTER(header),
    PROTO_GETTER(detections));

DEFINE_GET_PARSER(
    NavSatFix,
    PROTO_GETTER(header),
    PROTO_GETTER(status),
    PROTO_GETTER(latitude_deg),
    PROTO_GETTER(longitude_deg),
    PROTO_GETTER(altitude_m),
    PROTO_GETTER(position_covariance_m2),
    PROTO_GETTER(position_covariance_type));

DEFINE_GET_PARSER(Bool, PROTO_GETTER(data));

DEFINE_GET_PARSER(Byte, PROTO_GETTER(data));

DEFINE_GET_PARSER(Char, PROTO_GETTER(data));

DEFINE_GET_PARSER(Empty);

DEFINE_GET_PARSER(Float32, PROTO_GETTER(data));

DEFINE_GET_PARSER(Float64, PROTO_GETTER(data));

DEFINE_GET_PARSER(Int16, PROTO_GETTER(data));

DEFINE_GET_PARSER(Int32, PROTO_GETTER(data));

DEFINE_GET_PARSER(Int64, PROTO_GETTER(data));

DEFINE_GET_PARSER(Int8, PROTO_GETTER(data));

DEFINE_GET_PARSER(String, PROTO_GETTER(data));

DEFINE_GET_PARSER(UInt16, PROTO_GETTER(data));

DEFINE_GET_PARSER(UInt32, PROTO_GETTER(data));

DEFINE_GET_PARSER(UInt64, PROTO_GETTER(data));

DEFINE_GET_PARSER(UInt8, PROTO_GETTER(data));

template <typename Rng>
TransformStamped random_element(
    converter::TypeTag<TransformStamped> /*unused*/,
    InOut<Rng> rng) {
  TransformStamped result;

  result.mutable_header()->CopyFrom(converter::random_element<Header>(rng));
  result.set_child_frame_id(UUID::new_uuid().to_string());
  result.mutable_transform()->CopyFrom(
      converter::random_element<transforms::proto::SE3>(rng));
  // We don't use frame IDs when converting to/from ROS2.
  constexpr int DIMS = 3;
  result.mutable_transform()->mutable_into()->mutable_id()->set_data(
      transforms::Frame<DIMS>::null_frame().id().to_string());
  result.mutable_transform()->mutable_from()->mutable_id()->set_data(
      transforms::Frame<DIMS>::null_frame().id().to_string());

  return result;
}

template <typename Rng>
PoseWithCovariance random_element(
    converter::TypeTag<PoseWithCovariance> /*unused*/,
    InOut<Rng> rng) {
  PoseWithCovariance result;
  result.mutable_pose()->CopyFrom(
      converter::random_element<transforms::proto::SE3>(rng));

  // We don't use frame IDs when converting to/from ROS2.
  constexpr int DIMS = 3;
  result.mutable_pose()->mutable_into()->mutable_id()->set_data(
      transforms::Frame<DIMS>::null_frame().id().to_string());
  result.mutable_pose()->mutable_from()->mutable_id()->set_data(
      transforms::Frame<DIMS>::null_frame().id().to_string());

  constexpr std::size_t N = transforms::SE3::DOF;
  for (int ii = 0; ii < N * N; ++ii) {
    result.add_covariance(converter::random_element<double>(rng));
  }
  return result;
}

template <typename Rng>
Twist random_element(converter::TypeTag<Twist> /*unused*/, InOut<Rng> rng) {
  Twist result;
  constexpr std::size_t N = transforms::SE3::DOF;
  for (int ii = 0; ii < N; ++ii) {
    result.add_algebra(converter::random_element<double>(rng));
  }
  return result;
}

template <typename Rng>
TwistWithCovariance random_element(
    converter::TypeTag<TwistWithCovariance> /*unused*/,
    InOut<Rng> rng) {
  TwistWithCovariance result;
  result.mutable_twist()->CopyFrom(converter::random_element<Twist>(rng));
  constexpr std::size_t N = transforms::SE3::DOF;
  for (int ii = 0; ii < N * N; ++ii) {
    result.add_covariance(converter::random_element<double>(rng));
  }
  return result;
}

template <typename Rng>
ObjectHypothesis random_element(
    converter::TypeTag<ObjectHypothesis> /*unused*/,
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
Detection3D random_element(
    converter::TypeTag<Detection3D> /*unused*/,
    InOut<Rng> rng) {
  Detection3D result;
  result.mutable_header()->CopyFrom(converter::random_element<Header>(rng));

  auto bbox = converter::random_element<geometry::proto::OrientedBoxSE3>(rng);

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
    converter::TypeTag<BoundingBox2D> /*unused*/,
    InOut<Rng> rng) {
  BoundingBox2D result;
  result.set_center_x(converter::random_element<double>(rng));
  result.set_center_y(converter::random_element<double>(rng));
  result.set_theta_rad(converter::random_element<double>(rng));
  result.set_size_x(converter::random_element<double>(rng));
  result.set_size_y(converter::random_element<double>(rng));
  return result;
}

template <typename Rng>
Detection2D random_element(
    converter::TypeTag<Detection2D> /*unused*/,
    InOut<Rng> rng) {
  Detection2D result;
  result.mutable_header()->CopyFrom(converter::random_element<Header>(rng));
  constexpr int MIN_ELEMENTS = 1;
  constexpr int MAX_ELEMENTS = 10;
  std::uniform_int_distribution<int> dist{MIN_ELEMENTS, MAX_ELEMENTS};
  const int num_elements = dist(*rng);
  for (int ii = 0; ii < num_elements; ++ii) {
    result.add_results()->CopyFrom(
        converter::random_element<ObjectHypothesisWithPose>(rng));
  }

  result.mutable_bbox()->CopyFrom(
      converter::random_element<BoundingBox2D>(rng));
  result.set_id(UUID::new_uuid().to_string());
  return result;
}

template <typename Rng>
Detection3DArray random_element(
    converter::TypeTag<Detection3DArray> /*unused*/,
    InOut<Rng> rng) {
  Detection3DArray result;
  result.mutable_header()->CopyFrom(converter::random_element<Header>(rng));

  constexpr int MIN_ELEMENTS = 5;
  constexpr int MAX_ELEMENTS = 10;
  std::uniform_int_distribution<int> dist{MIN_ELEMENTS, MAX_ELEMENTS};
  const int num_elements = dist(*rng);
  for (int ii = 0; ii < num_elements; ++ii) {
    result.add_detections()->CopyFrom(
        converter::random_element<Detection3D>(rng));
  }

  return result;
}

template <typename Rng>
Detection2DArray random_element(
    converter::TypeTag<Detection2DArray> /*unused*/,
    InOut<Rng> rng) {
  Detection2DArray result;
  result.mutable_header()->CopyFrom(converter::random_element<Header>(rng));

  constexpr int MIN_ELEMENTS = 5;
  constexpr int MAX_ELEMENTS = 10;
  std::uniform_int_distribution<int> dist{MIN_ELEMENTS, MAX_ELEMENTS};
  const int num_elements = dist(*rng);
  for (int ii = 0; ii < num_elements; ++ii) {
    result.add_detections()->CopyFrom(
        converter::random_element<Detection2D>(rng));
  }

  return result;
}

template <typename Rng>
NavSatFix random_element(
    converter::TypeTag<NavSatFix> /*unused*/,
    InOut<Rng> rng) {
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
  result.mutable_header()->CopyFrom(converter::random_element<Header>(rng));
  result.set_status(STATUSES.at(
      converter::random_element<std::size_t>(rng) % STATUSES.size()));
  result.set_latitude_deg(converter::random_element<double>(rng));
  result.set_longitude_deg(converter::random_element<double>(rng));
  result.set_altitude_m(converter::random_element<double>(rng));

  // Make a valid covariance:
  Eigen::Matrix3d covariance{testing::random_matrix<Eigen::Matrix3d>(*rng)};
  covariance = covariance.transpose() * covariance;

  for (int ii = 0; ii < covariance.rows(); ++ii) {
    for (int jj = 0; jj < covariance.cols(); ++jj) {
      result.add_position_covariance_m2(covariance(ii, jj));
    }
  }
  result.set_position_covariance_type(COVARIANCES.at(
      converter::random_element<std::size_t>(rng) % COVARIANCES.size()));
  return result;
}

template <typename Rng>
Byte random_element(converter::TypeTag<Byte> /*unused*/, InOut<Rng> rng) {
  Byte result;
  result.set_data(std::string{converter::random_element<char>(rng)});
  return result;
}

template <typename Rng>
UInt8 random_element(converter::TypeTag<UInt8> /*unused*/, InOut<Rng> rng) {
  UInt8 result;
  result.set_data(std::string{converter::random_element<char>(rng)});
  return result;
}

template <typename Rng>
Char random_element(converter::TypeTag<Char> /*unused*/, InOut<Rng> rng) {
  Char result;
  result.set_data(std::string{converter::random_element<char>(rng)});
  return result;
}

template <typename Rng>
Bool random_element(converter::TypeTag<Bool> /*unused*/, InOut<Rng> rng) {
  std::uniform_int_distribution dist{0, 1};
  Bool result;
  result.set_data((dist(*rng) % 2) == 0);
  return result;
}

template <typename Rng>
Float32 random_element(converter::TypeTag<Float32> /*unused*/, InOut<Rng> rng) {
  Float32 result;
  result.set_data(converter::random_element<float>(rng));
  return result;
}

template <typename Rng>
Float64 random_element(converter::TypeTag<Float64> /*unused*/, InOut<Rng> rng) {
  Float64 result;
  result.set_data(converter::random_element<double>(rng));
  return result;
}

template <typename Rng>
Int16 random_element(converter::TypeTag<Int16> /*unused*/, InOut<Rng> rng) {
  Int16 result;
  set_data(converter::random_element<int16_t>(rng), InOut{result});
  return result;
}

template <typename Rng>
Int32 random_element(converter::TypeTag<Int32> /*unused*/, InOut<Rng> rng) {
  Int32 result;
  result.set_data(converter::random_element<int32_t>(rng));
  return result;
}

template <typename Rng>
Int64 random_element(converter::TypeTag<Int64> /*unused*/, InOut<Rng> rng) {
  Int64 result;
  result.set_data(converter::random_element<int64_t>(rng));
  return result;
}

template <typename Rng>
Int8 random_element(converter::TypeTag<Int8> /*unused*/, InOut<Rng> rng) {
  Int8 result;
  result.set_data(std::string{converter::random_element<int8_t>(rng)});
  return result;
}

template <typename Rng>
String random_element(converter::TypeTag<String> /*unused*/, InOut<Rng> rng) {
  String result;
  result.set_data(UUID::new_uuid().to_string());
  return result;
}

template <typename Rng>
UInt16 random_element(converter::TypeTag<UInt16> /*unused*/, InOut<Rng> rng) {
  UInt16 result;
  set_data(converter::random_element<int16_t>(rng), InOut{result});
  return result;
}

template <typename Rng>
UInt32 random_element(converter::TypeTag<UInt32> /*unused*/, InOut<Rng> rng) {
  UInt32 result;
  result.set_data(converter::random_element<uint32_t>(rng));
  return result;
}

template <typename Rng>
UInt64 random_element(converter::TypeTag<UInt64> /*unused*/, InOut<Rng> rng) {
  UInt64 result;
  result.set_data(converter::random_element<uint64_t>(rng));
  return result;
}

bool custom_verify_equality(
    const PoseWithCovariance &a,
    const PoseWithCovariance &b);

bool custom_verify_equality(const Twist &a, const Twist &b);

bool custom_verify_equality(
    const TwistWithCovariance &a,
    const TwistWithCovariance &b);

}  // namespace resim::msg
