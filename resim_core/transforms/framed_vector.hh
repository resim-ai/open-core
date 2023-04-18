#pragma once

#include <Eigen/Dense>
#include <utility>

#include "resim_core/assert/assert.hh"
#include "resim_core/transforms/frame.hh"

namespace resim::transforms {

// FramedVector
// Use this class to associate a D-dimensional vector with a Frame<D> object.
// It is assumed that the root of the vector is at the origin of the Frame<D>
// and that the vector is expressed in the coordinates of the Frame<D>.
//
// These can be useful for expressing and summing (for example): forces on a
// body's centre of mass or wind velocities on a body's centre of pressure. Or -
// more generally - any application where it is helpful to keep track of the
// specific frame in which a vector is expressed and check that we are not
// mistakenly summing vectors in different frames.
//
// Attempts to sum two framed vectors will fail if they are not in the same
// frame. FramedVector<D> inherits from an Eigen Vector with dimensionality D
// thus operations with unframed Eigen vectors are supported - including
// addition. However with no ability to verify the frames.
template <const unsigned dims>
class FramedVector : public Eigen::Matrix<double, dims, 1> {
 public:
  using Vector = Eigen::Matrix<double, dims, 1>;
  static constexpr unsigned DIMS = dims;

  // FramedVectors are built from an appropriate Eigen vector and a frame of
  // the same dimensionality.
  // @param[in] vector - A DIMS dimensional Eigen Vector.
  // @param[in] frame - The frame in which the vector is expressed.
  FramedVector(Vector vector, Frame<dims> frame)
      : Vector(std::move(vector)),
        frame_(std::move(frame)) {}

  // FramedVectors can be build from an Eigen vector alone and a new frame will
  // be generated.
  // @param[in] vector - A DIMS dimensional Eigen Vector.
  explicit FramedVector(Vector vector)
      : Vector(std::move(vector)),
        frame_(Frame<dims>::new_frame()) {}

  FramedVector() = default;

  // Summing two framed vectors is just like summing two regular vectors, only
  // we check that the frames match.
  // @param[in] other - A FramedVector to add to this one.
  // @returns - A FramedVector representing the sum with the same frame as this.
  FramedVector<dims> operator+(const FramedVector<dims> &other) const {
    REASSERT(frame_ == other.frame_, "Vector frames must match");
    return FramedVector<dims>(Vector::operator+(other), frame_);
  }

  FramedVector<dims> operator*(double d) const {
    return FramedVector<dims>(Vector::operator*(d), frame_);
  }

  friend FramedVector<dims> operator*(double d, const FramedVector &v) {
    return FramedVector<dims>(d * v.vector(), v.frame());
  }

  // Summing with a matching unframed vector returns another unframed vector
  // @param[in] other - An unframed Vector to add to this one.
  // @returns - An unframed Vector representing the sum.
  Vector operator+(const Vector &other) const {
    return Vector::operator+(other);
  }

  // Equality comparison operator that considers the frame as well
  // as the vector.
  // Note that comparisons between unframed and framed vectors still work,
  // through inheritance from the base class, although they ignore frames.
  bool operator==(const FramedVector<dims> &other) const {
    return Vector::operator==(other) && frame_ == other.frame();
  }

  // The approximate equality comparison method also considers frames.
  // Note that we have intentionally broken the ReSim style guide to match the
  // syntax of the method in the Eigen base class. Otherwise we would have
  // confusing syntax differences depending on whether the vector is on the
  // left or the right. Example:
  // Eigen::Vector3d A;
  // FramedVector<3> B(A);
  // A.isApprox(B);
  // B.is_approx(A); // Confusing syntactic switch.
  template <typename... Args>
  bool isApprox(const FramedVector<dims> &other, Args &&...args) const {
    return Vector::isApprox(other, std::forward<Args>(args)...) &&
           frame_ == other.frame();
  }

  // Because the overload of isApprox above clobbers the base class method,
  // we must redeclare this method here in order to forward comparisons with
  // non-FramedVector types to the base class method.
  template <typename... Args>
  bool isApprox(Args &&...args) const {
    return Vector::isApprox(std::forward<Args>(args)...);
  }

  void set_frame(const Frame<dims> &frame) { frame_ = frame; }

  const Frame<dims> &frame() const { return frame_; }

  const Vector &vector() const { return *this; }

 private:
  Frame<dims> frame_;
};

}  // namespace resim::transforms
