// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/msg/fuzz_helpers.hh"

template <typename...>
struct Foo;
namespace resim::msg {
bool custom_verify_equality(
    const PoseWithCovariance &a,
    const PoseWithCovariance &b) {
  if (not converter::verify_equality(a.pose(), b.pose())) {
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
    if (not converter::verify_equality(a.covariance(ii), b.covariance(ii))) {
      return false;
    }
  }
  return true;
}

bool custom_verify_equality(const Twist &a, const Twist &b) {
  constexpr std::size_t N = transforms::SE3::DOF;
  REASSERT(a.algebra().size() == N, "Incorrect twist size detected!");
  REASSERT(b.algebra().size() == N, "Incorrect twist size detected!");

  for (int ii = 0; ii < N; ++ii) {
    if (not converter::verify_equality(a.algebra(ii), b.algebra(ii))) {
      return false;
    }
  }
  return true;
}

bool custom_verify_equality(
    const TwistWithCovariance &a,
    const TwistWithCovariance &b) {
  const bool twists_equal = converter::verify_equality(a.twist(), b.twist());
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
    if (not converter::verify_equality(a.covariance(ii), b.covariance(ii))) {
      return false;
    }
  }
  return true;
}

}  // namespace resim::msg
