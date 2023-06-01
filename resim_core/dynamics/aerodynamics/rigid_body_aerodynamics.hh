#pragma once

#include <Eigen/Dense>
#include <functional>
#include <initializer_list>
#include <memory>
#include <utility>
#include <vector>

#include "resim_core/actor/state/rigid_body_state.hh"
#include "resim_core/transforms/frame.hh"
#include "resim_core/transforms/framed_vector.hh"
#include "resim_core/transforms/se3.hh"
#include "resim_core/utils/nullable_reference.hh"

namespace resim::dynamics::aerodynamics {

// This is a pure virtual class, that provides references to input "state" for
// an AerodynamicElement. An example of this would be an aileron angle, or flap
// angle. Such state should not be stored within the AerodynamicElement itself,
// which is intended to be a largely "functional" element. (If you're used to
// the strategy pattern, these correspond to the strategy.)
//
// In practice, we expect this state to also include variables more naturally
// associated with controls - these are all just considered "state" to the
// AerodynamicElement implementation.
struct AerodynamicElementState {
  AerodynamicElementState() = default;
  AerodynamicElementState(AerodynamicElementState const &other) = default;
  AerodynamicElementState(AerodynamicElementState &&other) = default;
  AerodynamicElementState &operator=(AerodynamicElementState const &other) =
      default;
  AerodynamicElementState &operator=(AerodynamicElementState &&other) = default;
  virtual ~AerodynamicElementState() = default;
};

// An AerodynamicElement models a component of a rigid aerodynamic object as a
// centre-of-pressure (CoP) frame, and an aerodynamics function that will
// calculate the pressure force vector on the CoP (in the CoP frame) given
// the apparent wind vector (also in the CoP frame), the current RigidBodyState
// of center of mass (CoM), and the ambient wind (in
// the reference frame.) In addition, a state associated to the individual
// element can be passed in using AerodynamicElementState. The apparent wind in
// the CoM or CoP frame is then calculated using the class methods.

// This is an abstract base class for this object, in order to allow different
// types of state inputs. The CRTP implementation of this class is
// AerodynamicElementImpl. It's important to use that implementation if you
// expect to use RigidBodyAerodynamics, which can combine multiple
// AerodynamicElements of different types.
class AerodynamicElement {
 public:
  // Supporting only SE3 for now.
  static constexpr unsigned DIM = transforms::SE3::DIMS;
  static constexpr unsigned DOF = transforms::SE3::DOF;
  using FramedVector = transforms::FramedVector<DIM>;
  using TangentVector = typename transforms::SE3::TangentVector;

  AerodynamicElement(AerodynamicElement const &other) = default;
  AerodynamicElement(AerodynamicElement &&other) = default;
  AerodynamicElement &operator=(AerodynamicElement const &other) = default;
  AerodynamicElement &operator=(AerodynamicElement &&other) = default;

  virtual ~AerodynamicElement() = default;

  // Construct an aerodynamic element
  // param[in] com_from_cop - An SE3 transform between the center-of-pressure
  //                          (CoP) frame of the element and the center-of-mass
  //                          (CoM) frame of the parent body.
  explicit AerodynamicElement(transforms::SE3 com_from_cop);

  // Returns the apparent wind vector in the CoP frame given the ambient wind.
  // and the motion of the body as described by the body state.
  // param[in] body_com_state - The rigid body state represented as an SE3
  //                            between the body CoM frame and a reference
  //                            frame and also the transform derivatives.
  // param[in] ref_local_wind - The wind speed vector at the CoM,
  //                            expressed in the reference frame.
  FramedVector cop_local_wind(
      const actor::state::RigidBodyState<transforms::SE3> &body_com_state,
      const FramedVector &ref_local_wind) const;

  // Retuns the pressure force vector on the CoP, in the CoP frame.
  // param[in] body_com_state - The rigid body state represented as an SE3
  //                            between the body CoM frame and a reference
  //                            frame and also the transform derivatives.
  // param[in] ref_local_wind - The wind speed vector at the CoM,
  //                            expressed in the reference frame.
  // param[in] aerodynamic_state - Abstract state of aerodynamic element.
  FramedVector cop_local_force(
      const actor::state::RigidBodyState<transforms::SE3> &body_com_state,
      const FramedVector &ref_local_wind,
      const AerodynamicElementState &aerodynamic_state) const;

  // Returns the pressure torque & force TangentVector on the body CoM, due
  // to the pressure force on the CoP.
  // param[in] body_com_state - The rigid body state represented as an SE3
  //                            between the body CoM frame and a reference
  //                            frame and also the transform derivatives.
  // param[in] ref_local_wind - The wind speed vector at the CoM,
  //                            expressed in the reference frame.
  // param[in] aerodynamic_state - Abstract state of aerodynamic element.
  TangentVector com_force(
      const actor::state::RigidBodyState<transforms::SE3> &body_com_state,
      const FramedVector &ref_local_wind,
      const AerodynamicElementState &aerodynamic_state) const;

  const transforms::SE3 &com_from_cop() const;
  const transforms::Frame<DIM> &com_frame() const;
  const transforms::Frame<DIM> &cop_frame() const;

 protected:
  // Abstract function which takes wind in CoP frame, and state of the element,
  // and returns the force on the CoP, also in CoP frame.
  // param[in] wind - The wind speed vector at the CoP, expressed in the CoP
  //                  frame.
  // param[in] aerodynamic_state - Abstract state of aerodynamic element.
  //
  // In practice, this should be implemented via AerodynamicElementImpl.
  //
  // TODO(tknowles): In future work, this should return a SE3::TangentVector,
  // and CoP should change to aerodynamic center.
  virtual FramedVector aerodynamics_(
      const FramedVector &wind,
      const AerodynamicElementState &aerodynamic_state) const = 0;

 private:
  transforms::SE3 com_from_cop_;
};

// CRTP mix-in, allowing different types of state input to the
// AerodynamicElement class. Impl should always be the class name, for example:
//
// AirfoilElement : public AerodynamicElementImpl<AirfoilElement, AirfoilState>
//
// This class should then implement protected method:
//   static FramedVector aerodynamics_impl_(
//       const FramedVector &cop_local_wind,
//       const AirfoilState &state);
//
// This class then simply dispatches to that class method, based off
// the Impl type, by dynamically casting the state type to AirfoilState.
template <class Impl, class ElementStateType>
class AerodynamicElementImpl : public AerodynamicElement {
  using AerodynamicElement::AerodynamicElement;

 protected:
  FramedVector aerodynamics_(
      const FramedVector &wind,
      const AerodynamicElementState &element_state) const override {
    const auto element_state_ptr =
        dynamic_cast<const ElementStateType *>(&element_state);
    REASSERT(
        element_state_ptr != nullptr,
        "Casting AerodynamicElementState to specific type failed.");
    return static_cast<const Impl *>(this)->aerodynamics_impl_(
        wind,
        *element_state_ptr);
  }
};

// Models the aerodynamic pressure force on a rigid body, by breaking the body
// up into a list of AerodynamicElement components, each of which can be
// modelled as a center of pressure point. AerodynamicElements do not have to be
// of the same type.
class RigidBodyAerodynamics {
 public:
  // Supporting only SE3 for now.
  static constexpr unsigned DIM = transforms::SE3::DIMS;
  static constexpr unsigned DOF = transforms::SE3::DOF;
  using FramedVector = transforms::FramedVector<DIM>;
  using TangentVector = typename transforms::SE3::TangentVector;
  using Control = Eigen::VectorXd;

  // Construct a RigidBodyAerodynamics element from a well-formed vector of
  // AerodynamicElement components. All components must have the same
  // center-of-mass (CoM) frame. We allow forwarding the vector here for
  // performance, but there is no Frame checking. The other constructors do
  // check the frames, so use these if you desire more safety.
  //
  // param[in] components - A vector of AerodynamicElement components that make
  //                        up a rigid body.
  explicit RigidBodyAerodynamics(
      std::vector<std::shared_ptr<AerodynamicElement>> &&components);

  // Construct a RigidBodyAerodynamics element from a brace-enclosed list of
  // AerodynamicElement components. All components must have the same
  // center-of-mass (CoM) frame. This will be checked.
  //
  // param[in] components - A list of AerodynamicElement components that make up
  //                        a rigid body.
  RigidBodyAerodynamics(
      std::initializer_list<std::shared_ptr<AerodynamicElement>> components);

  // Append an AerodynamicElement components. All components must have the same
  // center-of-mass (CoM) frame. This will be checked.
  //
  // param[in] component - An new AerodynamicElement component.

  void append(std::shared_ptr<AerodynamicElement> component);

  // Compute the pressure force on the body in the CoM frame.
  //
  // param[in] body_com_state - The current state (position and derivatives) of
  //                            the body.
  // param[in] ref_local_wind - The vector of ambient wind speed local to the
  //                            body, expressed in the coordinates of the
  //                            reference frame.
  // param[in] component_states - Vector of element state, aligned to
  //                              the RigidBodyAerodynamic components (as
  //                              provided in the constructor.)
  TangentVector body_pressure_force(
      const actor::state::RigidBodyState<transforms::SE3> &body_com_state,
      const FramedVector &ref_local_wind,
      const std::vector<std::reference_wrapper<const AerodynamicElementState>>
          &component_states) const;

  // TODO(https://app.asana.com/0/1204081589980528/1204332537822012/f)
  // Instead of passing ref_local_wind as a 'known' argument, we should let
  // RigidBodyAerodynamics own a pointer to a provider function that looks up
  // the wind based on the position of the body. A similar provider could be
  // added to model prop-wash, where relevant.

  const std::vector<std::shared_ptr<AerodynamicElement>> &components() const;
  const transforms::Frame<DIM> &com_frame() const;

 private:
  std::vector<std::shared_ptr<AerodynamicElement>> components_;
};

}  // namespace resim::dynamics::aerodynamics
