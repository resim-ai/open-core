//
// liegroup_concepts.hh
//
// This file contains a collection of type traits that implement all of the
// member functions necessary to be a Lie Group. For a class `MyLieGroup`, we
// expect it to inherit an instantiation of the LieGroup template, and
// implement the following member functions:
//
// ================================================================================
// class MyGroup : public LieGroup<M, N> {
//  public:
//   // Get an identity MyGroup
//   static MyGroup identity();
//
//   // Operator*
//   // Compose this MyGroup with another (multiplication)
//   MyGroup operator*(const MyGroup &other) const;
//
//   // Operator*
//   // Apply the MyGroup action to a vector in 3-Dimensional space
//   // (multiplication)
//   Eigen::Vector<double, DIMS> operator*(
//       const Eigen::Vector3d &source_vector) const;
//
//   // Return the inverse of this MyGroup.
//   MyGroup inverse() const;
//
//   // Return the length of the geodesic curve between frames in the
//   // transformation.
//   double arc_length() const;
//
//   // Interpolate this MyGroup
//   // [param] fraction - interpolation is over a unit interval, where
//   // fraction=0 returns identity and fraction=1 returns this MyGroup. In
//   // between the MyGroup returned is a linear interpolation. If fraction is
//   // greater than 1 or less than 0, a linear extrapolation will be returned.
//   MyGroup interp(double fraction) const;
//
//   // Create an MyGroup from an element of the LieGroup algebra.
//   static MyGroup exp(const TangentVector &alg);
//
//   // Retrieve the element of the LieGroup algebra that represents
//   // this group element.
//   TangentVector log() const;
//
//   // Adjoint representation of this group element.
//   TangentMapping adjoint() const;
//
//   // Adjoint representation of a given algebra element.
//   static TangentMapping adjoint(const TangentVector &alg);
//
//   // Transform a TangentVector from the right tangent space to the left.
//   TangentVector adjoint_times(const TangentVector &alg) const;
//
//   // Adjoint times for algebra elements.
//   static TangentVector adjoint_times(
//       const TangentVector &alg_0,
//       const TangentVector &alg_1);
//
//   // Test for floating-point equality with another MyGroup.
//   bool is_approx(const MyGroup &other) const;
// };
// ================================================================================
//
// While one could use virtual functions to enforce this, we don't use this
// approach because:
//
//     1. We don't need to own LieGroups by base class pointer or reference.
//
//     2. Many of these methods require use of the derived type which means
//        that CRTP would need to be employed. This makes it significantly
//        more difficult to work with base classes in many contexts
//        (e.g. making MixIns for such classes is difficult).
//
#pragma once

#include <Eigen/Dense>
#include <concepts>
#include <type_traits>

#include "transforms/liegroup.hh"

namespace resim::transforms {

// Trait to check whether a type T supports composition.
template <typename T>
concept has_composition = requires(const T v) {
  { v* v } -> std::same_as<T>;
};

// Trait to check whether a type T has an identity element.
template <typename T>
concept has_identity = requires() {
  { T::identity() } -> std::same_as<T>;
};

// Trait to check whether a type T has an inverse.
template <typename T>
concept has_inverse = requires(const T v) {
  { v.inverse() } -> std::same_as<T>;
};

// Trait to check whether a type T inherits from LieGroup<DIMS, DOF>.
template <typename T>
concept inherits_liegroup = std::is_base_of_v<LieGroup<T::DIMS, T::DOF>, T>;

// Trait to check whether a type T has group action on R^DIM defined.
template <typename T>
concept has_action =
    requires(const T v, const Eigen::Matrix<double, T::DIMS, 1> point) {
  { v* point } -> std::same_as<Eigen::Matrix<double, T::DIMS, 1>>;
};

// Trait to check whether a type T has an exponential defined for it.
template <typename T>
concept has_exp = requires(const typename T::TangentVector v) {
  { T::exp(v) } -> std::same_as<T>;
};

// Trait to check whether a type T has a logarithm defined for it.
template <typename T>
concept has_log = requires(const T v) {
  { v.log() } -> std::same_as<typename T::TangentVector>;
};

// Trait to check whether a type T has an interpolation method defined for it.
template <typename T>
concept has_interp = requires(const T v, double x) {
  { v.interp(x) } -> std::same_as<T>;
};

// Trait to check whether a type T defines an adjoint method.
template <typename T>
concept has_adjoint = requires(const T v) {
  { v.adjoint() } -> std::same_as<typename T::TangentMapping>;
};

// Trait to check  whether a type T defines an adjoint_times() member function
template <typename T>
concept has_adjoint_times =
    requires(const T v, const typename T::TangentMapping dv) {
  { v.adjoint_times(dv) } -> std::same_as<typename T::TangentVector>;
};

// Trait to check whether a type T defines an algebra adjoint method.
template <typename T>
concept has_algebra_adjoint = requires(const typename T::TangentVector dv) {
  { T::adjoint(dv) } -> std::same_as<typename T::TangentMapping>;
};

// Trait to check if a type T defines is_approx() as a member function.
template <typename T>
concept has_is_approx = requires(const T v) {
  { v.is_approx(v) } -> std::same_as<bool>;
};

// clang-format off
template <typename T>
concept LieGroupType =
    has_composition<T>     &&
    has_identity<T>        &&
    has_inverse<T>         &&
    inherits_liegroup<T>   &&
    has_action<T>          &&
    has_exp<T>             &&
    has_interp<T>          &&
    has_log<T>             &&
    has_adjoint<T>         &&
    has_algebra_adjoint<T> &&  
    has_adjoint_times<T>   &&
    has_is_approx<T>;
// clang-format on  

}  // namespace resim::transforms
