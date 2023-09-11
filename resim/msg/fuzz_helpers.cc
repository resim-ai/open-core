// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/msg/fuzz_helpers.hh"

namespace resim::msg {

bool verify_equality(const Header &a, const Header &b) {
  return resim::verify_equality(a.stamp(), b.stamp()) and
         a.frame_id() == b.frame_id();
}

bool verify_equality(const TransformStamped &a, const TransformStamped &b) {
  return verify_equality(a.header(), b.header()) and
         a.child_frame_id() == b.child_frame_id() and
         verify_equality(a.transform(), b.transform());
}

bool verify_equality(const TransformArray &a, const TransformArray &b) {
  if (a.transforms_size() != b.transforms_size()) {
    return false;
  }
  for (int ii = 0; ii < a.transforms_size(); ++ii) {
    if (not verify_equality(a.transforms(ii), b.transforms(ii))) {
      return false;
    }
  }
  return true;
}

bool verify_equality(const PoseWithCovariance &a, const PoseWithCovariance &b) {
  if (not verify_equality(a.pose(), b.pose())) {
    return false;
  }

  constexpr std::size_t N = transforms::SE3::DOF;
  REASSERT(
      a.covariance().size() == N * N,
      "Incorrect covariance size detected!");
  REASSERT(
      b.covariance().size() == N * N,
      "Incorrect covariance size detected!");

  for (int ii = 0; ii < N * N; ++ii) {
    if (not resim::verify_equality(a.covariance(ii), b.covariance(ii))) {
      return false;
    }
  }
  return true;
}

bool verify_equality(const Twist &a, const Twist &b) {
  constexpr std::size_t N = transforms::SE3::DOF;
  REASSERT(a.algebra().size() == N, "Incorrect twist size detected!");
  REASSERT(b.algebra().size() == N, "Incorrect twist size detected!");

  for (int ii = 0; ii < N; ++ii) {
    if (not resim::verify_equality(a.algebra(ii), b.algebra(ii))) {
      return false;
    }
  }
  return true;
}

bool verify_equality(
    const TwistWithCovariance &a,
    const TwistWithCovariance &b) {
  const bool twists_equal = verify_equality(a.twist(), b.twist());
  if (not twists_equal) {
    return false;
  }

  constexpr std::size_t N = transforms::SE3::DOF;
  REASSERT(
      a.covariance().size() == N * N,
      "Incorrect covariance size detected!");
  REASSERT(
      b.covariance().size() == N * N,
      "Incorrect covariance size detected!");

  for (int ii = 0; ii < N * N; ++ii) {
    if (not resim::verify_equality(a.covariance(ii), b.covariance(ii))) {
      return false;
    }
  }
  return true;
}

bool verify_equality(const Odometry &a, const Odometry &b) {
  return verify_equality(a.header(), b.header()) and
         a.child_frame_id() == b.child_frame_id() and
         verify_equality(a.pose(), b.pose()) and
         verify_equality(a.twist(), b.twist());
}

bool verify_equality(const Detection3D &a, const Detection3D &b) {
  return verify_equality(a.header(), b.header()) and
         verify_equality(a.bbox(), b.bbox()) and a.id() == b.id();
}

bool verify_equality(const BoundingBox2D &a, const BoundingBox2D &b) {
  return resim::verify_equality(a.center_x(), b.center_x()) and
         resim::verify_equality(a.center_y(), b.center_y()) and
         resim::verify_equality(a.theta_rad(), b.theta_rad()) and
         resim::verify_equality(a.size_x(), b.size_x()) and
         resim::verify_equality(a.size_y(), b.size_y());
}

bool verify_equality(const Detection2D &a, const Detection2D &b) {
  return verify_equality(a.header(), b.header()) and
         verify_equality(a.bbox(), b.bbox()) and a.id() == b.id();
}

bool verify_equality(const Detection3DArray &a, const Detection3DArray &b) {
  if (not verify_equality(a.header(), b.header()) or
      a.detections_size() != b.detections_size()) {
    return false;
  }
  for (int ii = 0; ii < a.detections_size(); ++ii) {
    if (not verify_equality(a.detections(ii), b.detections(ii))) {
      return false;
    }
  }
  return true;
}

bool verify_equality(const Detection2DArray &a, const Detection2DArray &b) {
  if (not verify_equality(a.header(), b.header()) or
      a.detections_size() != b.detections_size()) {
    return false;
  }
  for (int ii = 0; ii < a.detections_size(); ++ii) {
    if (not verify_equality(a.detections(ii), b.detections(ii))) {
      return false;
    }
  }
  return true;
}

bool verify_equality(const NavSatFix &a, const NavSatFix &b) {
  constexpr int COV_DIM = 9;
  for (int ii = 0; ii < COV_DIM; ++ii) {
    if (not resim::verify_equality(
            a.position_covariance_m2(ii),
            b.position_covariance_m2(ii))) {
      return false;
    }
  }
  return verify_equality(a.header(), b.header()) and
         a.status() == b.status() and
         resim::verify_equality(a.latitude_deg(), b.latitude_deg()) and
         resim::verify_equality(a.longitude_deg(), b.longitude_deg()) and
         resim::verify_equality(a.altitude_m(), b.altitude_m()) and
         a.position_covariance_type() == b.position_covariance_type();
}
}  // namespace resim::msg
