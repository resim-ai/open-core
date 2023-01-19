#include "resim_core/actor/state/rigid_body_state.hh"

#include <gtest/gtest.h>

#include <random>
#include <utility>

#include "resim_core/curves/two_jet.hh"
#include "resim_core/testing/random_matrix.hh"
#include "resim_core/transforms/framed_group.hh"
#include "resim_core/transforms/liegroup_concepts.hh"
#include "resim_core/transforms/se3.hh"

namespace resim::actor::state {

namespace {

template <transforms::LieGroupType Group, typename Rng>
Group random_group_member(Rng &&rng) {
  return Group::exp(testing::random_vector<typename Group::TangentVector>(
      std::forward<Rng>(rng)));
}

template <transforms::LieGroupType Group, typename Rng>
curves::TwoJetR<Group> random_two_jet(Rng &&rng) {
  return curves::TwoJetR<Group>{
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

using RigidTransformTypes = ::testing::Types<transforms::FSE3, transforms::SE3>;
TYPED_TEST_SUITE(RigidBodyStateTests, RigidTransformTypes);

template <typename T>
class UnframedRigidBodyStateTests : public RigidBodyStateTests<T> {};
using UnframedTypes = ::testing::Types<transforms::SE3>;
TYPED_TEST_SUITE(UnframedRigidBodyStateTests, UnframedTypes);

template <typename T>
class FramedRigidBodyStateTests : public RigidBodyStateTests<T> {};
using FramedTypes = ::testing::Types<transforms::FSE3>;
TYPED_TEST_SUITE(FramedRigidBodyStateTests, FramedTypes);

TYPED_TEST(RigidBodyStateTests, TestConstructionFromGroup) {
  // SETUP / ACTION
  const auto ref_from_body = random_group_member<TypeParam>(this->rng_);
  const RigidBodyState<TypeParam> state(ref_from_body);
  // VERIFICATION
  EXPECT_TRUE(state.ref_from_body().is_approx(ref_from_body));
  EXPECT_TRUE(state.body_linear_velocity_mps().isZero());
  EXPECT_TRUE(state.body_angular_velocity_radps().isZero());
  EXPECT_TRUE(state.body_linear_acceleration_mpss().isZero());
  EXPECT_TRUE(state.body_angular_acceleration_radpss().isZero());
}

TYPED_TEST(RigidBodyStateTests, TestConstructionFromStateData) {
  // SETUP / ACTION
  const curves::TwoJetR<TypeParam> test_two_jet{
      random_two_jet<TypeParam>(this->rng_)};
  const RigidBodyState<TypeParam> state_a{test_two_jet};
  // Unpack the RigidBodyState to a ref_from_body transform and derivatives
  // in the body frame, then re-pack into a new state via the constructor.
  const RigidBodyState<TypeParam> state_b(
      state_a.ref_from_body(),
      state_a.body_derivatives());

  // VERIFICATION
  // The underlying TwoJets of the two RigidBodyStates should be the same.
  EXPECT_TRUE(state_a.ref_from_body_two_jet().is_approx(
      state_b.ref_from_body_two_jet()));
}

TYPED_TEST(RigidBodyStateTests, TestTwoJetConstruction) {
  // SETUP / ACTION
  const RigidBodyState<TypeParam> default_rigid_body_state;

  const curves::TwoJetR<TypeParam> test_two_jet{
      random_two_jet<TypeParam>(this->rng_)};
  const RigidBodyState<TypeParam> rigid_body_state{test_two_jet};

  // VERIFICATION
  const curves::TwoJetR<TypeParam> default_two_jet;
  EXPECT_TRUE(default_rigid_body_state.ref_from_body_two_jet().is_approx(
      default_two_jet));
  EXPECT_TRUE(rigid_body_state.ref_from_body_two_jet().is_approx(test_two_jet));
}

TYPED_TEST(RigidBodyStateTests, TestIdentity) {
  const auto identity_state = RigidBodyState<TypeParam>::identity();

  // Twojet is identity
  EXPECT_TRUE(identity_state.ref_from_body_two_jet().is_approx(
      curves::TwoJetR<TypeParam>::identity()));
}

TYPED_TEST(RigidBodyStateTests, TestBodyGetters) {
  // SETUP
  const curves::TwoJetR<TypeParam> test_two_jet{
      random_two_jet<TypeParam>(this->rng_)};
  const RigidBodyState<TypeParam> rigid_body_state{test_two_jet};

  // ACTION / VERIFICATION
  EXPECT_TRUE(test_two_jet.ref_from_frame().is_approx(
      rigid_body_state.ref_from_body()));
  EXPECT_TRUE(test_two_jet.d_ref_from_frame().isApprox(
      TypeParam::tangent_vector_from_parts(
          rigid_body_state.body_angular_velocity_radps(),
          rigid_body_state.body_linear_velocity_mps())));
  EXPECT_TRUE(test_two_jet.d_ref_from_frame().isApprox(
      TypeParam::tangent_vector_from_parts(
          rigid_body_state.body_derivatives().velocity.angular_radps,
          rigid_body_state.body_derivatives().velocity.linear_mps)));
  EXPECT_TRUE(test_two_jet.d2_ref_from_frame().isApprox(
      TypeParam::tangent_vector_from_parts(
          rigid_body_state.body_angular_acceleration_radpss(),
          rigid_body_state.body_linear_acceleration_mpss())));
  EXPECT_TRUE(test_two_jet.d2_ref_from_frame().isApprox(
      TypeParam::tangent_vector_from_parts(
          rigid_body_state.body_derivatives().acceleration.angular_radpss,
          rigid_body_state.body_derivatives().acceleration.linear_mpss)));
}

TYPED_TEST(RigidBodyStateTests, TestSetters) {
  // SETUP
  RigidBodyState<TypeParam> rigid_body_state;

  const TypeParam new_ref_from_body{random_group_member<TypeParam>(this->rng_)};
  const Eigen::Vector3d new_body_linear_velocity_mps{
      testing::random_vector<Eigen::Vector3d>(this->rng_)};
  const Eigen::Vector3d new_body_angular_velocity_radps{
      testing::random_vector<Eigen::Vector3d>(this->rng_)};
  const Eigen::Vector3d new_body_linear_acceleration_mpss{
      testing::random_vector<Eigen::Vector3d>(this->rng_)};
  const Eigen::Vector3d new_body_angular_acceleration_radpss{
      testing::random_vector<Eigen::Vector3d>(this->rng_)};

  // ACTION
  rigid_body_state.set_ref_from_body(new_ref_from_body);
  rigid_body_state.set_body_linear_velocity_mps(new_body_linear_velocity_mps);
  rigid_body_state.set_body_angular_velocity_radps(
      new_body_angular_velocity_radps);
  rigid_body_state.set_body_linear_acceleration_mpss(
      new_body_linear_acceleration_mpss);
  rigid_body_state.set_body_angular_acceleration_radpss(
      new_body_angular_acceleration_radpss);

  // VERIFICATION
  EXPECT_TRUE(new_ref_from_body.is_approx(rigid_body_state.ref_from_body()));
  EXPECT_EQ(
      new_body_linear_velocity_mps,
      rigid_body_state.body_linear_velocity_mps());
  EXPECT_EQ(
      new_body_angular_velocity_radps,
      rigid_body_state.body_angular_velocity_radps());
  EXPECT_EQ(
      new_body_linear_acceleration_mpss,
      rigid_body_state.body_linear_acceleration_mpss());
  EXPECT_EQ(
      new_body_angular_acceleration_radpss,
      rigid_body_state.body_angular_acceleration_radpss());
}

TYPED_TEST(UnframedRigidBodyStateTests, OperatorTest) {
  const curves::TwoJetR<TypeParam> test_two_jet_a{
      random_two_jet<TypeParam>(this->rng_)};
  const curves::TwoJetR<TypeParam> test_two_jet_b{
      random_two_jet<TypeParam>(this->rng_)};
  const RigidBodyState<TypeParam> state_a{test_two_jet_a};
  const RigidBodyState<TypeParam> state_b{test_two_jet_b};

  const auto state_c = state_a * state_b;

  EXPECT_TRUE(state_c.ref_from_body_two_jet().is_approx(
      state_a.ref_from_body_two_jet() * state_b.ref_from_body_two_jet()));
}

TYPED_TEST(FramedRigidBodyStateTests, OperatorTest) {
  const auto FRAME_A = transforms::Frame<TypeParam::DIMS>::new_frame();
  const auto FRAME_B = transforms::Frame<TypeParam::DIMS>::new_frame();
  const auto FRAME_C = transforms::Frame<TypeParam::DIMS>::new_frame();
  curves::TwoJetR<TypeParam> test_two_jet_a{
      random_two_jet<TypeParam>(this->rng_)};
  auto temp_group_a = random_group_member<TypeParam>(this->rng_);
  temp_group_a.set_into(FRAME_A);
  temp_group_a.set_from(FRAME_B);
  test_two_jet_a.set_ref_from_frame(temp_group_a);
  curves::TwoJetR<TypeParam> test_two_jet_b{
      random_two_jet<TypeParam>(this->rng_)};
  auto temp_group_b = random_group_member<TypeParam>(this->rng_);
  temp_group_b.set_into(FRAME_B);
  temp_group_b.set_from(FRAME_C);
  test_two_jet_b.set_ref_from_frame(temp_group_b);
  const RigidBodyState<TypeParam> state_a{test_two_jet_a};
  const RigidBodyState<TypeParam> state_b{test_two_jet_b};

  const auto state_c = state_a * state_b;

  EXPECT_TRUE(state_c.ref_from_body_two_jet().is_approx(
      state_a.ref_from_body_two_jet() * state_b.ref_from_body_two_jet()));
  EXPECT_EQ(state_c.ref_from_body().into(), FRAME_A);
  EXPECT_EQ(state_c.ref_from_body().from(), FRAME_C);
  EXPECT_NE(state_c.ref_from_body().from(), FRAME_B);
}

TYPED_TEST(UnframedRigidBodyStateTests, TestInverseTimes) {
  // SETUP
  const curves::TwoJetR<TypeParam> test_two_jet_a{
      random_two_jet<TypeParam>(this->rng_)};
  const curves::TwoJetR<TypeParam> test_two_jet_b{
      random_two_jet<TypeParam>(this->rng_)};
  const curves::TwoJetR<TypeParam> test_two_jet{
      random_two_jet<TypeParam>(this->rng_)};
  const RigidBodyState<TypeParam> state_a{test_two_jet_a};
  const RigidBodyState<TypeParam> state_b{test_two_jet_b};

  const auto state_c = state_a.inverse_times(state_b);

  // ACTION / VERIFICATION
  EXPECT_TRUE(state_c.ref_from_body_two_jet().is_approx(
      test_two_jet_a.inverse() * test_two_jet_b));
}

TYPED_TEST(FramedRigidBodyStateTests, InverseTimes) {
  const auto FRAME_A = transforms::Frame<TypeParam::DIMS>::new_frame();
  const auto FRAME_B = transforms::Frame<TypeParam::DIMS>::new_frame();
  const auto FRAME_C = transforms::Frame<TypeParam::DIMS>::new_frame();
  curves::TwoJetR<TypeParam> test_two_jet_a{
      random_two_jet<TypeParam>(this->rng_)};
  auto temp_group_a = random_group_member<TypeParam>(this->rng_);
  temp_group_a.set_into(FRAME_A);
  temp_group_a.set_from(FRAME_B);
  test_two_jet_a.set_ref_from_frame(temp_group_a);
  curves::TwoJetR<TypeParam> test_two_jet_b{
      random_two_jet<TypeParam>(this->rng_)};
  auto temp_group_b = random_group_member<TypeParam>(this->rng_);
  temp_group_b.set_into(FRAME_A);
  temp_group_b.set_from(FRAME_C);
  test_two_jet_b.set_ref_from_frame(temp_group_b);
  const RigidBodyState<TypeParam> state_a{test_two_jet_a};
  const RigidBodyState<TypeParam> state_b{test_two_jet_b};

  const auto state_c = state_a.inverse_times(state_b);

  EXPECT_TRUE(state_c.ref_from_body_two_jet().is_approx(
      state_a.ref_from_body_two_jet().inverse() *
      state_b.ref_from_body_two_jet()));
  EXPECT_EQ(state_c.ref_from_body().into(), FRAME_B);
  EXPECT_EQ(state_c.ref_from_body().from(), FRAME_C);
  EXPECT_NE(state_c.ref_from_body().from(), FRAME_A);
}

}  // namespace resim::actor::state
