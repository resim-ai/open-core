#pragma once

#include <Eigen/Dense>

#include "resim_core/time/timestamp.hh"
#include "resim_core/utils/nullable_reference.hh"

namespace resim::dynamics {

//
// This concept defines types which have the necessary properties to be used as
// a state within our dynamics, controller, and integrator frameworks. In
// particular, these states don't need to live in a vector space, but they must
// have an integer number of degrees of freedom and it must be possible to "add"
// a vector with that number of degrees of freedom to them. The important part
// about this plus operator is that it can be used to add perturbations to
// elements of the State manifold. For states that are in a vector space, this
// is just standard addition. For Lie Groups, this addition operator can be the
// right or left plus operator as defined in this document:
// https://drive.google.com/file/d/1UlI1N63o6abyL03VfbYoXu22CcYTdZ6b/view?usp=sharing
//
template <typename T>
concept StateType = requires(T t) {
  { T::DOF } -> std::convertible_to<int>;
  { t + Eigen::Matrix<double, T::DOF, 1>() } -> std::same_as<T>;
};

//
// This interface describes the dynamics for a system of ordinary differential
// equations (ODEs). In other words, classes that implement this interface
// represent f in the system of ODEs:
//
//                     (d/dt)[state] = f(state, control, time)
//                           control = u(state, time)
//
// The interface also gives implementers the ability to add Jacobian information
// that can be useful for integrating these dynamics implicitly, or for model
// predictive control. Not all dynamics are expected to implement this Jacobian
// functionality. This is a class because such dynamics are often parameterized
// by things like, e.g. mass or moments of inertia.
//
template <StateType State_t, int CONTROL_DIM>
class Dynamics {
 public:
  static constexpr int DOF = State_t::DOF;
  using State = State_t;

  // A delta or a TangentVector of the state type, as described above.
  using Delta = Eigen::Matrix<double, DOF, 1>;

  // The control vector type
  using Control = Eigen::Matrix<double, CONTROL_DIM, 1>;

  // Matrix aliases used for Jacobians.
  using MatXX = Eigen::Matrix<double, DOF, DOF>;
  using MatXU = Eigen::Matrix<double, DOF, CONTROL_DIM>;

  // Struct of the Jacobians of the dynamics f with respect to the state (x) and
  // the control (u).
  struct Diffs {
    MatXX f_x{MatXX::Zero()};
    MatXU f_u{MatXU::Zero()};
  };

  Dynamics() = default;
  Dynamics(const Dynamics &) = default;
  Dynamics(Dynamics &&) noexcept = default;
  Dynamics &operator=(const Dynamics &) = default;
  Dynamics &operator=(Dynamics &&) noexcept = default;
  virtual ~Dynamics() = default;

  // The dynamics themselves.
  // @param[in] state - The current state.
  // @param[in] control - The control vector.
  // @param[in] time - The current time.
  // @param[out] diffs - The Jacobians of the dynamics with respect to state and
  //                     control. This should be populated if not null.
  virtual Delta operator()(
      const State &state,
      const Control &control,
      time::Timestamp time,
      NullableReference<Diffs> diffs) const = 0;
};

}  // namespace resim::dynamics
