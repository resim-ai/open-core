#include "resim_core/visualization/view_server/view_server_test_helper.hh"

#include "resim_core/actor/state/trajectory.hh"
#include "resim_core/assert/assert.hh"
#include "resim_core/curves/d_curve.hh"
#include "resim_core/curves/d_curve_test_helpers.hh"
#include "resim_core/curves/t_curve.hh"
#include "resim_core/transforms/frame.hh"
#include "resim_core/transforms/framed_group.hh"
#include "resim_core/transforms/liegroup_test_helpers.hh"
#include "resim_core/transforms/se3.hh"
#include "resim_core/transforms/so3.hh"
#include "resim_core/visualization/curve/test_helpers.hh"

namespace resim::visualization::view_server {

namespace {
using transforms::FSE3;
using transforms::FSO3;
using transforms::SE3;
using transforms::SO3;
using Frame = transforms::Frame<3>;

constexpr auto LOW_COUNT =
    "The minimum number of test elements you can request is seven. Please "
    "increase the count.";

constexpr auto TRANSFORM_PREFIX = "transform";
constexpr auto ROTATION_PREFIX = "rotation";
constexpr auto D_CURVE_PREFIX = "d_curve";
constexpr auto T_CURVE_PREFIX = "t_curve";
constexpr auto TRAJECTORY_PREFIX = "trajectory";

constexpr time::Timestamp ZERO_TIME;
}  // namespace

template <>
std::vector<Frame> generate_payload_type(const unsigned count) {
  // How many test elements to make.
  REASSERT(count >= detail::MIN_TEST_ELEMENTS, LOW_COUNT);
  std::vector<Frame> frames;
  frames.reserve(count);
  for (int i = 0; i < count; i++) {
    Frame frame{Frame::new_frame()};
    frames.push_back(frame);
  }
  return frames;
}

template <>
std::vector<SE3> generate_payload_type(const unsigned count) {
  // How many test elements to make.
  REASSERT(count >= detail::MIN_TEST_ELEMENTS, LOW_COUNT);
  return transforms::make_test_group_elements<SE3>(count);
}

template <>
std::vector<SO3> generate_payload_type(const unsigned count) {
  // How many test elements to make.
  REASSERT(count >= detail::MIN_TEST_ELEMENTS, LOW_COUNT);
  return transforms::make_test_group_elements<SO3>(count);
}

template <>
std::vector<FSE3> generate_payload_type(const unsigned count) {
  // How many test elements to make.
  REASSERT(count >= detail::MIN_TEST_ELEMENTS, LOW_COUNT);
  return transforms::make_test_group_elements<FSE3>(count);
}

template <>
std::vector<FSO3> generate_payload_type(const unsigned count) {
  // How many test elements to make.
  REASSERT(count >= detail::MIN_TEST_ELEMENTS, LOW_COUNT);
  return transforms::make_test_group_elements<FSO3>(count);
}

template <>
std::vector<curves::DCurve<SE3>> generate_payload_type(const unsigned count) {
  // How many test elements to make.
  REASSERT(count >= detail::MIN_TEST_ELEMENTS, LOW_COUNT);
  std::vector<curves::DCurve<SE3>> d_curves;
  d_curves.reserve(count);

  for (int i = 0; i < count; i++) {
    curves::DCurve<SE3> curve(curves::DCurveCircle<SE3>::points());
    d_curves.push_back(curve);
  }

  return d_curves;
}

template <>
std::vector<curves::DCurve<FSE3>> generate_payload_type(const unsigned count) {
  // How many test elements to make.
  REASSERT(count >= detail::MIN_TEST_ELEMENTS, LOW_COUNT);
  std::vector<curves::DCurve<FSE3>> d_curves;
  d_curves.reserve(count);

  for (int i = 0; i < count; i++) {
    curves::DCurve<FSE3> curve(curves::DCurveCircle<FSE3>::points());
    d_curves.push_back(curve);
  }
  return d_curves;
}

template <>
std::vector<curves::TCurve<transforms::FSE3>> generate_payload_type(
    const unsigned count) {
  // How many test elements to make.
  REASSERT(count >= detail::MIN_TEST_ELEMENTS, LOW_COUNT);
  std::vector<curves::TCurve<transforms::FSE3>> t_curves;
  const transforms::Frame<3> into{transforms::Frame<3>::new_frame()};
  const transforms::Frame<3> from{transforms::Frame<3>::new_frame()};
  t_curves.reserve(count);

  for (int i = 0; i < count; i++) {
    const curves::TCurve<transforms::FSE3> test_curve{
        curve::testing::make_circle_curve(into, from)};
    t_curves.push_back(test_curve);
  }

  return t_curves;
}

template <>
std::vector<actor::state::Trajectory> generate_payload_type(
    const unsigned count) {
  REASSERT(count >= detail::MIN_TEST_ELEMENTS, LOW_COUNT);
  std::vector<actor::state::Trajectory> trajectories;
  std::vector<curves::TCurve<transforms::FSE3>> t_curves =
      generate_payload_type<curves::TCurve<transforms::FSE3>>(count);
  trajectories.reserve(count);

  for (int i = 0; i < count; i++) {
    const actor::state::Trajectory test_trajectory{t_curves.at(i), ZERO_TIME};
    trajectories.push_back(test_trajectory);
  }

  return trajectories;
}

template <>
std::string get_type_prefix<SE3>() {
  return TRANSFORM_PREFIX;
}

template <>
std::string get_type_prefix<SO3>() {
  return ROTATION_PREFIX;
}

template <>
std::string get_type_prefix<FSE3>() {
  return TRANSFORM_PREFIX;
}

template <>
std::string get_type_prefix<FSO3>() {
  return ROTATION_PREFIX;
}

template <>
std::string get_type_prefix<curves::DCurve<SE3>>() {
  return D_CURVE_PREFIX;
}

template <>
std::string get_type_prefix<curves::DCurve<FSE3>>() {
  return D_CURVE_PREFIX;
}

template <>
std::string get_type_prefix<curves::TCurve<FSE3>>() {
  return T_CURVE_PREFIX;
}

template <>
std::string get_type_prefix<actor::state::Trajectory>() {
  return TRAJECTORY_PREFIX;
}

}  // namespace resim::visualization::view_server
