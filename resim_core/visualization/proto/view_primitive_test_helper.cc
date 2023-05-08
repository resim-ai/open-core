#include "resim_core/visualization/proto/view_primitive_test_helper.hh"

#include <optional>
#include <random>
#include <string>

#include "resim_core/actor/state/trajectory.hh"
#include "resim_core/curves/d_curve.hh"
#include "resim_core/curves/t_curve.hh"
#include "resim_core/curves/test_helpers.hh"
#include "resim_core/testing/random_matrix.hh"
#include "resim_core/time/timestamp.hh"
#include "resim_core/transforms/frame.hh"
#include "resim_core/transforms/framed_group.hh"
#include "resim_core/transforms/framed_vector.hh"
#include "resim_core/transforms/liegroup_test_helpers.hh"
#include "resim_core/transforms/se3.hh"
#include "resim_core/transforms/so3.hh"
#include "resim_core/visualization/view_primitive.hh"

namespace resim::visualization {

namespace {
using transforms::FSE3;
using transforms::FSO3;
using transforms::SE3;
using transforms::SO3;
using Frame = transforms::Frame<3>;
using FramedVector = transforms::FramedVector<3>;
constexpr unsigned int SEED = 42;
constexpr unsigned int NUM_GROUP_POINTS = 10;
constexpr time::Timestamp ZERO_TIME;
}  // namespace

template <typename Rng>
curves::TCurve<FSE3> generate_test_t_curve(Rng&& rng) {
  auto control_point_poses =
      transforms::make_test_group_elements<FSE3>(NUM_GROUP_POINTS);
  const Frame into{Frame::new_frame()};
  const Frame from{Frame::new_frame()};
  std::vector<curves::TCurve<FSE3>::Control> control_points;
  control_points.reserve(NUM_GROUP_POINTS);
  double time = 0;
  for (FSE3& pose : control_point_poses) {
    using TwoJet = curves::TwoJetL<FSE3>;
    pose.set_into(into);
    pose.set_from(from);
    const TwoJet point{
        pose,
        testing::random_vector<FSE3::TangentVector>(rng),
        testing::random_vector<FSE3::TangentVector>(rng)};
    control_points.push_back(curves::TCurve<FSE3>::Control{
        .time = time,
        .point = point,
    });
    time += 1.;
  }
  return curves::TCurve<FSE3>{control_points};
}

// Generates a test object for a given resim type
template <>
Frame generate_test_object() {
  return Frame::new_frame();
}

template <>
SE3 generate_test_object() {
  // Make random seed deterministic
  std::mt19937 rng{SEED};
  const SE3::TangentVector test_tangent{
      testing::random_vector<SE3::TangentVector>(rng)};
  return SE3::exp(test_tangent);
}

template <>
SO3 generate_test_object() {
  // Make random seed deterministic
  std::mt19937 rng{SEED};
  const SO3::TangentVector test_tangent{
      testing::random_vector<SO3::TangentVector>(rng)};
  return SO3::exp(test_tangent);
}

template <>
FSE3 generate_test_object() {
  // Make random seed deterministic
  std::mt19937 rng{SEED};
  const Frame into{Frame::new_frame()};
  const Frame from{Frame::new_frame()};
  const FSE3::TangentVector test_tangent{
      testing::random_vector<FSE3::TangentVector>(rng)};
  return FSE3::exp(test_tangent, into, from);
}

template <>
FSO3 generate_test_object() {
  // Make random seed deterministic
  std::mt19937 rng{SEED};
  const Frame into{Frame::new_frame()};
  const Frame from{Frame::new_frame()};
  const FSO3::TangentVector test_tangent{
      testing::random_vector<FSO3::TangentVector>(rng)};
  return FSO3::exp(test_tangent, into, from);
}

template <>
curves::DCurve<SE3> generate_test_object() {
  return curves::DCurve(
      transforms::make_test_group_elements<SE3>(NUM_GROUP_POINTS));
}

template <>
curves::DCurve<FSE3> generate_test_object() {
  return curves::DCurve(
      transforms::make_test_group_elements<FSE3>(NUM_GROUP_POINTS));
}

template <>
curves::TCurve<FSE3> generate_test_object() {
  // Make random seed deterministic
  std::mt19937 rng{SEED};
  return generate_test_t_curve(rng);
}

template <>
FramedVector generate_test_object() {
  std::mt19937 rng{SEED};
  return FramedVector{testing::random_vector<Eigen::Matrix<double, 3, 1>>(rng)};
}

template <>
actor::state::Trajectory generate_test_object() {
  // Make random seed deterministic
  std::mt19937 rng{SEED};
  curves::TCurve<FSE3> t_curve = generate_test_t_curve(rng);
  return actor::state::Trajectory{t_curve, ZERO_TIME};
}

template <typename T>
ViewPrimitive generate_test_primitive(const std::optional<std::string>& name) {
  ViewPrimitive test_primitive{
      .id = UUID::new_uuid(),
      .payload = generate_test_object<T>(),
      .user_defined_name = name,
      .file_name = detail::TEST_FILE_NAME,
      .line_number = detail::TEST_LINE_NUMBER,
  };

  return test_primitive;
}

template ViewPrimitive generate_test_primitive<Frame>(
    const std::optional<std::string>& name);
template ViewPrimitive generate_test_primitive<SE3>(
    const std::optional<std::string>& name);
template ViewPrimitive generate_test_primitive<SO3>(
    const std::optional<std::string>& name);
template ViewPrimitive generate_test_primitive<FSE3>(
    const std::optional<std::string>& name);
template ViewPrimitive generate_test_primitive<FSO3>(
    const std::optional<std::string>& name);
template ViewPrimitive generate_test_primitive<curves::DCurve<SE3>>(
    const std::optional<std::string>& name);
template ViewPrimitive generate_test_primitive<curves::DCurve<FSE3>>(
    const std::optional<std::string>& name);
template ViewPrimitive generate_test_primitive<curves::TCurve<FSE3>>(
    const std::optional<std::string>& name);
template ViewPrimitive generate_test_primitive<actor::state::Trajectory>(
    const std::optional<std::string>& name);
template ViewPrimitive generate_test_primitive<FramedVector>(
    const std::optional<std::string>& name);
}  // namespace resim::visualization
