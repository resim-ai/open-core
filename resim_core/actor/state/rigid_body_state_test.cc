#include "resim_core/actor/state/rigid_body_state.hh"

#include <gtest/gtest.h>

#include <random>
#include <utility>

#include "resim_core/curves/two_jet.hh"
#include "resim_core/transforms/framed_group.hh"
#include "resim_core/transforms/liegroup_concepts.hh"
#include "resim_core/transforms/se3.hh"
#include "resim_core/utils/random_matrix.hh"

namespace resim::actor::state {

namespace {

template <transforms::LieGroupType Group, typename Rng>
Group random_group_member(Rng &&rng) {
  return Group::exp(testing::random_vector<typename Group::TangentVector>(
      std::forward<Rng>(rng)));
}

template <transforms::LieGroupType Group, typename Rng>
curves::TwoJet<Group> random_two_jet(Rng &&rng) {
  return curves::TwoJet<Group>{
      random_group_member<Group>(std::forward<Rng>(rng)),
      testing::random_vector<typename Group::TangentVector>(
          std::forward<Rng>(rng)),
      testing::random_vector<typename Group::TangentVector>(
          std::forward<Rng>(rng))};
}

}  // namespace

template <typename T>
class RigidBodyStateTests : public ::testing::Test {
 protected:
  static constexpr unsigned SEED = 6954U;
  std::mt19937 rng_{SEED};
};

using RigidTransformTypes = ::testing::Types<transforms::SE3, transforms::FSE3>;
TYPED_TEST_SUITE(RigidBodyStateTests, RigidTransformTypes);

TYPED_TEST(RigidBodyStateTests, TestConstruction) {
  // SETUP / ACTION
  const RigidBodyState<TypeParam> default_rigid_body_state;

  const curves::TwoJet<TypeParam> test_two_jet{
      random_two_jet<TypeParam>(this->rng_)};
  const RigidBodyState<TypeParam> rigid_body_state{test_two_jet};

  // VERIFICATION
  const curves::TwoJet<TypeParam> default_two_jet;
  EXPECT_TRUE(default_rigid_body_state.body_from_reference_two_jet().is_approx(
      default_two_jet));
  EXPECT_TRUE(
      rigid_body_state.body_from_reference_two_jet().is_approx(test_two_jet));
}

TYPED_TEST(RigidBodyStateTests, TestGetters) {
  // SETUP
  const curves::TwoJet<TypeParam> test_two_jet{
      random_two_jet<TypeParam>(this->rng_)};
  const RigidBodyState<TypeParam> rigid_body_state{test_two_jet};

  // ACTION / VERIFICATION
  EXPECT_TRUE(test_two_jet.frame_from_ref().is_approx(
      rigid_body_state.body_from_reference()));
  EXPECT_TRUE(test_two_jet.frame_from_ref().is_approx(
      rigid_body_state.reference_from_body().inverse()));
  EXPECT_TRUE(test_two_jet.frame_from_ref().inverse().translation().isApprox(
      rigid_body_state.position_m()));
  EXPECT_TRUE(test_two_jet.frame_from_ref().inverse().rotation().log().isApprox(
      rigid_body_state.orientation_rad()));
  EXPECT_TRUE(test_two_jet.d_frame_from_ref().isApprox(
      TypeParam::tangent_vector_from_parts(
          -rigid_body_state.angular_velocity_radps(),
          -rigid_body_state.linear_velocity_mps())));
  EXPECT_TRUE(test_two_jet.d2_frame_from_ref().isApprox(
      TypeParam::tangent_vector_from_parts(
          -rigid_body_state.angular_acceleration_radpss(),
          -rigid_body_state.linear_acceleration_mpss())));
}

TYPED_TEST(RigidBodyStateTests, TestSetters) {
  // SETUP
  RigidBodyState<TypeParam> rigid_body_state;

  const TypeParam new_ref_from_body{random_group_member<TypeParam>(this->rng_)};
  const Eigen::Vector3d new_linear_velocity_mps{
      testing::random_vector<Eigen::Vector3d>(this->rng_)};
  const Eigen::Vector3d new_angular_velocity_radps{
      testing::random_vector<Eigen::Vector3d>(this->rng_)};
  const Eigen::Vector3d new_linear_acceleration_mpss{
      testing::random_vector<Eigen::Vector3d>(this->rng_)};
  const Eigen::Vector3d new_angular_acceleration_radpss{
      testing::random_vector<Eigen::Vector3d>(this->rng_)};

  // ACTION
  rigid_body_state.set_reference_from_body(new_ref_from_body);
  rigid_body_state.set_linear_velocity_mps(new_linear_velocity_mps);
  rigid_body_state.set_angular_velocity_radps(new_angular_velocity_radps);
  rigid_body_state.set_linear_acceleration_mpss(new_linear_acceleration_mpss);
  rigid_body_state.set_angular_acceleration_radpss(
      new_angular_acceleration_radpss);

  // VERIFICATION
  EXPECT_TRUE(
      new_ref_from_body.is_approx(rigid_body_state.reference_from_body()));
  EXPECT_EQ(new_linear_velocity_mps, rigid_body_state.linear_velocity_mps());
  EXPECT_EQ(
      new_angular_velocity_radps,
      rigid_body_state.angular_velocity_radps());
  EXPECT_EQ(
      new_linear_acceleration_mpss,
      rigid_body_state.linear_acceleration_mpss());
  EXPECT_EQ(
      new_angular_acceleration_radpss,
      rigid_body_state.angular_acceleration_radpss());
}

TYPED_TEST(RigidBodyStateTests, TestIdentity) {
  // SETUP / ACTION
  const RigidBodyState<TypeParam> identity{
      RigidBodyState<TypeParam>::identity()};

  // VERIFICATION
  EXPECT_TRUE(identity.body_from_reference_two_jet().is_approx(
      curves::TwoJet<TypeParam>::identity()));
}

}  // namespace resim::actor::state
