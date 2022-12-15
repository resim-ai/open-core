#pragma once

#include <Eigen/Dense>

#include "resim_core/curves/two_jet.hh"
#include "resim_core/transforms/liegroup_concepts.hh"

namespace resim::actor::state {

template <transforms::LieGroupType Group>
class RigidBodyState {
 public:
  static constexpr unsigned int DIMS = Group::DIMS;
  static constexpr unsigned int DOF = Group::DOF;

  // Angular dimension of the group
  static constexpr unsigned int ANG_DIM = DOF - DIMS;

  using LinearDelta = Eigen::Matrix<double, DIMS, 1>;
  using AngularDelta = Eigen::Matrix<double, ANG_DIM, 1>;

  RigidBodyState() = default;
  explicit RigidBodyState(curves::TwoJet<Group> body_from_reference);

  // Construct a RigidBodyState containing the identity transform and zero
  // derivatives.
  static RigidBodyState identity();

  // Getter for the body from reference transform.
  const Group &body_from_reference() const;
  Group reference_from_body() const;

  // Get the position of the body in body coordinates
  LinearDelta position_m() const;

  // Body orientation in angle-axis form
  AngularDelta orientation_rad() const;

  // Getters for linear and angular acceleration in body coordinates
  LinearDelta linear_velocity_mps() const;
  AngularDelta angular_velocity_radps() const;
  LinearDelta linear_acceleration_mpss() const;
  AngularDelta angular_acceleration_radpss() const;

  // Getter for the raw two jet
  const curves::TwoJet<Group> &body_from_reference_two_jet() const;

  // Setter for the reference_from_body transform
  void set_reference_from_body(const Group &reference_from_body);

  // Setters for linear and angular acceleration in body coordinates
  void set_linear_velocity_mps(LinearDelta linear_velocity_mps);
  void set_angular_velocity_radps(AngularDelta angular_velocity_radps);
  void set_linear_acceleration_mpss(LinearDelta linear_acceleration_mpss);
  void set_angular_acceleration_radpss(
      AngularDelta angular_acceleration_radpss);

 private:
  curves::TwoJet<Group> body_from_reference_;
};

}  // namespace resim::actor::state
