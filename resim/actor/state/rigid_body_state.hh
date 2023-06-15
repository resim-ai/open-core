#pragma once

#include <Eigen/Dense>

#include "resim/curves/two_jet.hh"
#include "resim/transforms/liegroup_concepts.hh"

namespace resim::actor::state {

template <transforms::LieGroupType Group>
class RigidBodyState {
 public:
  static constexpr unsigned int DIMS = Group::DIMS;
  static constexpr unsigned int DOF = Group::DOF;

  // Angular dimension of the group
  static constexpr unsigned int ANG_DIM = DOF - DIMS;

  using LinearDelta = Eigen::Matrix<double, DIMS, 1>;
  using LinearDeltaBlock =
      Eigen::VectorBlock<const typename Group::TangentVector, DIMS>;
  using AngularDelta = Eigen::Matrix<double, ANG_DIM, 1>;
  using AngularDeltaBlock =
      Eigen::VectorBlock<const typename Group::TangentVector, ANG_DIM>;

  // First derivatives.
  struct Velocity {
    LinearDelta linear_mps = LinearDelta::Zero();
    AngularDelta angular_radps = AngularDelta::Zero();
  };

  // Second derivatives.
  struct Acceleration {
    LinearDelta linear_mpss = LinearDelta::Zero();
    AngularDelta angular_radpss = AngularDelta::Zero();
  };

  // First and second derivatives.
  struct StateDerivatives {
    Velocity velocity;
    Acceleration acceleration;
  };

  RigidBodyState() = default;
  // Construct a Rigid body state from a pose. Time derivatives will be zero.
  // @param[in] ref_from_body - Group element representing the transform
  //                            to reference coordinates from body coordinates.
  //                            The first and second time derivatives will be
  //                            initialized as zero.
  explicit RigidBodyState(Group ref_from_body);

  // Construct a Rigid body state from a pose and time derivatives.
  // @param[in] ref_from_body - Group element representing the transform
  //                            to reference coordinates from body coordinates.
  // @param[in] body_derivatives - First and second time derivatives of the
  //                               transformation in right tangent space. That
  //                               is, the velocity and acceleration of the body
  //                               in the body frame.
  RigidBodyState(Group ref_from_body, StateDerivatives body_derivatives);

  // Construct a Rigid body state directly from a right TwoJet.
  // @param[in] body_from_ref - A TwoJetR holding a transform from reference
  //                            coordinates to body coordinates and the time
  //                            derivatives in right tangent space.
  // Use this constructor if you happen to have a valid TwoJetR at hand. If
  // you are constructing this state from body velocities and accelerations,
  // then we recommend using the constructor above.
  explicit RigidBodyState(curves::TwoJetR<Group> ref_from_body);

  // Construct a RigidBodyState containing the identity transform and zero
  // derivatives.
  template <typename... Args>
  static RigidBodyState<Group> identity(Args &&...args);

  // Compose this RigidBodState with another.
  // Composition behaves very similarly to LieGroup composition. It is
  // non-commutative and inner-frames must match for a valid composition.
  // An example use might be:
  //    RigidBodyState<FE3> global_from_robot;
  //    RigidBodyState<SE3> robot_from_sensor;
  // And we would like to know the state of the sensor with reference to the
  // global frame. In this case we would do:
  // auto global_from_sensor =
  //    global_from_robot * robot_from_sensor;
  RigidBodyState<Group> operator*(const RigidBodyState<Group> &other) const;

  // Invert this Rigid body state and multiply it by another.
  // Please note that - by convention - RigidBodyState objects keep the
  // reference frame on the left and the body frame on the right. Therefore,
  // inverting it would create an 'unconventional' object with the
  // body frame on the left. However, a valid case would be when you want the
  // body frame of this state to be the reference frame for another.
  // For example, say we have two robots with states:
  //    RigidBodyState<SE3> global_from_robot_a;
  //    RigidBodyState<SE3> global_from_robot_b;
  // And we would like to understand robot_b's state from robot_a's perspective,
  // In other words we want robot_b's body state with robot_a's body state as
  // the reference frame. In this case we would do:
  // auto robot_a_from_robot_b =
  //    global_from_robot_a.inverse_times(global_from_robot_b);
  // Reminder, if you are using the SE3 group, then the frames will be checked
  // and invalid compositions will generate errors.
  RigidBodyState<Group> inverse_times(const RigidBodyState<Group> &other) const;

  // Getter for the body from reference transform.
  const Group &ref_from_body() const;

  // Getters for bundled derivatives.
  // @returns the time derivatives in the body frame.
  StateDerivatives body_derivatives() const;

  // TODO(https://app.asana.com/0/1202178773526279/1203706226829869/f)
  // Consider a helper to access left tangent space derivatives e.g. for
  // inspecting the state of the body the reference observer's frame.

  // Individual getters for linear and angular derivatives in body coordinates
  LinearDeltaBlock body_linear_velocity_mps() const;
  AngularDeltaBlock body_angular_velocity_radps() const;
  LinearDeltaBlock body_linear_acceleration_mpss() const;
  AngularDeltaBlock body_angular_acceleration_radpss() const;

  // Getter for the raw two jet
  const curves::TwoJetR<Group> &ref_from_body_two_jet() const;

  // Setter for the reference_from_body transform
  void set_ref_from_body(Group ref_from_body);

  // Setters for linear and angular acceleration in body coordinates
  void set_body_linear_velocity_mps(LinearDelta body_linear_velocity_mps);
  void set_body_angular_velocity_radps(
      AngularDelta body_angular_velocity_radps);
  void set_body_linear_acceleration_mpss(
      LinearDelta body_linear_acceleration_mpss);
  void set_body_angular_acceleration_radpss(
      AngularDelta body_angular_acceleration_radpss);

 private:
  curves::TwoJetR<Group> ref_from_body_;
};

}  // namespace resim::actor::state
