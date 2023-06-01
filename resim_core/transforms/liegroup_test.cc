#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <vector>

#include "liegroup_concepts.hh"
#include "resim_core/transforms/liegroup_test_helpers.hh"
#include "resim_core/transforms/se3.hh"
#include "resim_core/transforms/so3.hh"

namespace resim::transforms {

namespace {
// Create some test frames.
constexpr unsigned int DIMS = 3;
const Frame<DIMS> A = Frame<DIMS>::new_frame();
const Frame<DIMS> B = Frame<DIMS>::new_frame();
const Frame<DIMS> C = Frame<DIMS>::new_frame();
}  // namespace

// Common tests that apply over all LieGroup classes, for example: SO3, SE3.
// For specialized tests see the test files for the classes themselves, for
// example so3_test.cc
template <typename t>
class LieGroupTests : public ::testing::Test {};

using LieGroupTypes = ::testing::Types<SO3, SE3>;
TYPED_TEST_SUITE(LieGroupTests, LieGroupTypes);

TYPED_TEST(LieGroupTests, InverseNegativeAlgEquivalence) {
  for (const typename TypeParam::TangentVector &alg :
       make_test_algebra_elements<TypeParam>()) {
    const TypeParam b_from_a_ref = TypeParam::exp(alg).inverse();
    const TypeParam b_from_a = TypeParam::exp(-alg);
    EXPECT_TRUE(b_from_a_ref.is_approx(b_from_a));
  }
}

TYPED_TEST(LieGroupTests, InterpZeroIdentity) {
  // Confirm interpolating at zero gives identity.
  constexpr double ZERO = 0;
  for (const TypeParam &a_from_b : make_test_group_elements<TypeParam>()) {
    EXPECT_TRUE(a_from_b.interp(ZERO).is_approx(TypeParam::identity()));
  }
}

TYPED_TEST(LieGroupTests, InterpOneNoop) {
  // Confirm interpolating at one is a noop.
  constexpr double ONE = 1.;
  for (const TypeParam &a_from_b : make_test_group_elements<TypeParam>()) {
    EXPECT_TRUE(a_from_b.interp(ONE).is_approx(a_from_b));
  }
}

TYPED_TEST(LieGroupTests, ExpOfZero) {
  TypeParam a_from_a_ref = TypeParam::identity();
  TypeParam a_from_a = TypeParam::exp(TypeParam::TangentVector::Zero());
  EXPECT_TRUE(a_from_a_ref.is_approx(a_from_a));
}

TYPED_TEST(LieGroupTests, LogOfIdentity) {
  const TypeParam a_from_a = TypeParam::identity();
  // Test log of identity TypeParam is zero.
  EXPECT_TRUE(a_from_a.log().isApprox(TypeParam::TangentVector::Zero()));
}

TYPED_TEST(LieGroupTests, ExpOfLogNoop) {
  std::vector<TypeParam> test_elements = make_test_group_elements<TypeParam>();
  // Exp should always be the inverse of log.
  for (const TypeParam &a_from_b : test_elements) {
    const TypeParam a_from_b_ref = TypeParam::exp(a_from_b.log());
    EXPECT_TRUE(a_from_b_ref.is_approx(a_from_b));
  }
}

TYPED_TEST(LieGroupTests, SelfAdjointNoop) {
  for (const typename TypeParam::TangentVector &alg :
       make_test_algebra_elements<TypeParam>()) {
    const TypeParam a_from_b = TypeParam::exp(alg);
    const typename TypeParam::TangentVector alg_noop =
        a_from_b.adjoint_times(a_from_b.log());
    EXPECT_TRUE(alg.isApprox(alg_noop));
  }
}

TYPED_TEST(LieGroupTests, CompositionByAdjoint) {
  const auto test_elements = make_test_group_elements<TypeParam>();
  const TypeParam &a_from_b = test_elements.back();
  for (const TypeParam &b_from_c : test_elements) {
    const TypeParam a_from_c_ref = a_from_b * b_from_c;
    const TypeParam a_from_c =
        TypeParam::exp(a_from_b.adjoint_times(b_from_c.log())) * a_from_b;
    EXPECT_TRUE(a_from_c_ref.is_approx(a_from_c));
  }
}

TYPED_TEST(LieGroupTests, AdjointAndTimesGroup) {
  using TangentVector = typename TypeParam::TangentVector;
  const auto test_elements = make_test_group_elements<TypeParam>();
  const TangentVector alg = test_elements.back().log();
  for (const TypeParam &grp : test_elements) {
    const TangentVector adj_times = grp.adjoint_times(alg);
    const TangentVector adj_times_alt = grp.adjoint() * alg;
    EXPECT_TRUE(adj_times.isApprox(adj_times_alt));
  }
}

TYPED_TEST(LieGroupTests, AdjointAndTimesAlgebra) {
  using TangentVector = typename TypeParam::TangentVector;
  const auto test_alg_elements = make_test_algebra_elements<TypeParam>();
  const TangentVector &b = test_alg_elements.back();
  for (const TangentVector &a : test_alg_elements) {
    const TangentVector adj_times = TypeParam::adjoint_times(a, b);
    const TangentVector adj_times_alt = TypeParam::adjoint(a) * b;
    // isApprox struggles when vectors are close to zero.
    constexpr double TOLERANCE = 1e-9;
    EXPECT_TRUE((adj_times - adj_times_alt).isZero(TOLERANCE));
  }
}

TYPED_TEST(LieGroupTests, AlgebraAdjointAntiCommutative) {
  using TangentVector = typename TypeParam::TangentVector;
  const auto test_alg_elements = make_test_algebra_elements<TypeParam>();
  const TangentVector &b = test_alg_elements.back();
  for (const TangentVector &a : test_alg_elements) {
    const TangentVector x = TypeParam::adjoint_times(a, b);
    const TangentVector y = TypeParam::adjoint_times(b, a);
    // One of our test vectors is close to zero and isApprox struggles close to
    // zero.
    constexpr double TOLERANCE = 1e-9;
    EXPECT_TRUE((x + y).isZero(TOLERANCE));
  }
}

TYPED_TEST(LieGroupTests, FloatingPointEquality) {
  const auto test_elements = make_test_group_elements<TypeParam>();
  const TypeParam &a_from_a = test_elements.front();
  for (auto test_elements_it = test_elements.begin() + 1;
       test_elements_it < test_elements.end();
       ++test_elements_it) {
    const TypeParam &a_from_b = *test_elements_it;
    const TypeParam &a_from_b_cp = *test_elements_it;
    EXPECT_TRUE(a_from_b.is_approx(a_from_b_cp));
    // Given that all the test elements are guaranteed to be unique - a
    // constraint that is enforced in the test for the helper lib - then it is
    // reasonable to expect the below to test to return false consistently.
    EXPECT_FALSE(a_from_a.is_approx(a_from_b));
  }
}

template <typename Group>
using FramedLieGroupTests = LieGroupTests<Group>;
TYPED_TEST_SUITE(FramedLieGroupTests, LieGroupTypes);

template <typename Group>
using FramedLieGroupAssertionTests = LieGroupTests<Group>;
TYPED_TEST_SUITE(FramedLieGroupAssertionTests, LieGroupTypes);

TYPED_TEST(FramedLieGroupTests, Construction) {
  const TypeParam a_from_b =
      TypeParam::exp(TypeParam::TangentVector::Ones(), A, B);
  EXPECT_TRUE(a_from_b.is_approx(a_from_b));
  EXPECT_EQ(A, a_from_b.into());
  EXPECT_EQ(B, a_from_b.from());
}

TYPED_TEST(FramedLieGroupTests, Copying) {
  const TypeParam a_from_b =
      TypeParam::exp(TypeParam::TangentVector::Ones(), A, B);
  const TypeParam &copy_a_from_b(a_from_b);
  const TypeParam &copy2_a_from_b = a_from_b;
  EXPECT_EQ(A, copy_a_from_b.into());
  EXPECT_EQ(B, copy_a_from_b.from());
  EXPECT_EQ(A, copy2_a_from_b.into());
  EXPECT_EQ(B, copy2_a_from_b.from());
}

TYPED_TEST(FramedLieGroupTests, FrameSetters) {
  // SETUP
  TypeParam a_from_b = TypeParam::exp(TypeParam::TangentVector::Ones(), A, B);

  // VERIFICATION
  EXPECT_NE(C, a_from_b.into());
  EXPECT_NE(A, a_from_b.from());

  // ACTION
  a_from_b.set_frames(C, A);

  // VERIFICATION
  EXPECT_EQ(C, a_from_b.into());
  EXPECT_EQ(A, a_from_b.from());
  EXPECT_TRUE(a_from_b.is_framed());

  // ACTION
  a_from_b.set_unframed();

  // VERIFICATION
  EXPECT_FALSE(a_from_b.is_framed());
}

TYPED_TEST(FramedLieGroupAssertionTests, FrameSettingError) {
  // SETUP
  TypeParam a_from_b = TypeParam::exp(TypeParam::TangentVector::Ones());
  const Frame<DIMS> null_frame;
  EXPECT_THROW({ a_from_b.set_frames(A, null_frame); }, AssertException);
  EXPECT_THROW({ a_from_b.set_frames(null_frame, B); }, AssertException);
  EXPECT_THROW(
      { a_from_b.set_frames(null_frame, null_frame); },
      AssertException);
  EXPECT_NO_THROW({ a_from_b.set_frames(A, B); });

  EXPECT_THROW(
      { TypeParam bad_xform(TypeParam::identity(A, null_frame)); },
      AssertException);
  EXPECT_THROW(
      { TypeParam bad_xform(TypeParam::identity(null_frame, B)); },
      AssertException);
  EXPECT_NO_THROW({ TypeParam bad_xform(TypeParam::identity(A, B)); });
  EXPECT_NO_THROW(
      { TypeParam bad_xform(TypeParam::identity(null_frame, null_frame)); });
}

TYPED_TEST(FramedLieGroupTests, IdentityTransformNoFrames) {
  // Construct an Identity Group with no frames.
  const TypeParam null_from_null = TypeParam::identity();
  // Verify LieGroup matches underlying type identity.
  EXPECT_TRUE(null_from_null.is_approx_transform(TypeParam::identity()));
  // Verify the frames
  EXPECT_FALSE(null_from_null.is_framed());
  EXPECT_TRUE(null_from_null.into().is_null());
  EXPECT_TRUE(null_from_null.from().is_null());
  EXPECT_NE(A, null_from_null.into());
  EXPECT_NE(B, null_from_null.from());
  EXPECT_EQ(null_from_null.into(), null_from_null.from());
}

TYPED_TEST(FramedLieGroupTests, IdentityTransformWithFrames) {
  // Construct an Identity typed transform.
  const TypeParam a_from_b = TypeParam::identity(A, B);
  // Verify LieGroup matches underlying type identity.
  EXPECT_TRUE(a_from_b.is_approx_transform(TypeParam::identity()));
  // Verify the frames
  EXPECT_TRUE(a_from_b.is_framed());
  EXPECT_FALSE(a_from_b.into().is_null());
  EXPECT_FALSE(a_from_b.from().is_null());
  EXPECT_EQ(A, a_from_b.into());
  EXPECT_EQ(B, a_from_b.from());
  EXPECT_NE(a_from_b.into(), a_from_b.from());
}

TYPED_TEST(FramedLieGroupTests, Composition) {
  TypeParam unframed_a_from_b = TypeParam::identity();
  TypeParam a_from_b = TypeParam::identity(A, B);
  for (TypeParam b_from_c : make_test_group_elements<TypeParam>()) {
    b_from_c.set_frames(B, C);
    TypeParam unframed_b_from_c = b_from_c;
    unframed_b_from_c.set_unframed();
    // Multiply framed groups, and unframed groups
    const TypeParam a_from_c = a_from_b * b_from_c;
    const TypeParam unframed_a_from_c = unframed_a_from_b * unframed_b_from_c;
    // Verify products are the same.
    EXPECT_TRUE(unframed_a_from_c.is_approx_transform(a_from_c));
    // Verify frames;
    EXPECT_EQ(A, a_from_c.into());
    EXPECT_EQ(C, a_from_c.from());
    EXPECT_NE(B, a_from_c.from());
    // Update
    unframed_a_from_b = unframed_b_from_c;
    a_from_b = unframed_a_from_b;
    a_from_b.set_frames(A, B);
  }
}

// NOLINTBEGIN(readability-function-cognitive-complexity)
TYPED_TEST(FramedLieGroupTests, CompositionHeterogeneousTypes) {
  TypeParam unframed_a_from_b = TypeParam::identity();
  TypeParam a_from_b = TypeParam::identity(A, B);
  for (TypeParam b_from_c : make_test_group_elements<TypeParam>()) {
    b_from_c.set_frames(B, C);
    TypeParam unframed_b_from_c = b_from_c;
    unframed_b_from_c.set_unframed();

    const TypeParam unframed_a_from_c_0 = a_from_b * unframed_b_from_c;
    const TypeParam unframed_a_from_c_1 = unframed_a_from_b * b_from_c;

    EXPECT_FALSE(unframed_a_from_c_0.is_framed());
    EXPECT_FALSE(unframed_a_from_c_1.is_framed());

    const TypeParam a_from_c = a_from_b * b_from_c;
    EXPECT_EQ(a_from_c.into(), A);
    EXPECT_EQ(a_from_c.from(), C);
    EXPECT_TRUE(a_from_c.is_approx_transform(unframed_a_from_c_0));
    EXPECT_TRUE(a_from_c.is_approx_transform(unframed_a_from_c_1));
    // Update
    unframed_a_from_b = unframed_b_from_c;
    a_from_b = unframed_a_from_b;
    a_from_b.set_frames(A, B);
  }
}
// NOLINTEND(readability-function-cognitive-complexity)

TYPED_TEST(FramedLieGroupAssertionTests, InvalidComposition) {
  // Construct two transform.
  const TypeParam a_from_b =
      TypeParam::exp(TypeParam::TangentVector::Ones(), A, B);
  const TypeParam b_from_c = TypeParam::identity(B, C);
  // Check that an assertion fails for invalid composition.
  EXPECT_THROW({ b_from_c *a_from_b; }, AssertException);
}

TYPED_TEST(FramedLieGroupAssertionTests, InvalidCompositionReference) {
  // SETUP
  const TypeParam unframed_a_from_b{
      TypeParam::exp(TypeParam::TangentVector::Ones())};
  const TypeParam unframed_b_from_c{TypeParam::identity()};
  TypeParam a_from_b(unframed_a_from_b);
  a_from_b.set_frames(A, B);
  TypeParam b_from_c(unframed_b_from_c);
  b_from_c.set_frames(B, C);

  const TypeParam &a_from_b_ref{a_from_b};
  const TypeParam &b_from_c_ref{b_from_c};

  // ACTION / VERIFICATION
  EXPECT_THROW({ b_from_c_ref *a_from_b; }, AssertException);
  EXPECT_THROW({ b_from_c *a_from_b_ref; }, AssertException);
  EXPECT_THROW({ b_from_c_ref *a_from_b_ref; }, AssertException);

  // No death:
  EXPECT_TRUE((a_from_b_ref * b_from_c_ref).is_approx(a_from_b * b_from_c));
}

TYPED_TEST(FramedLieGroupTests, RotateFramedVectorTest) {
  // Construct an Identity typed transform.
  const TypeParam a_from_b(TypeParam::identity(A, B));
  const FramedVector<TypeParam::DIMS> test_vector(Eigen::Vector3d::Ones(), B);
  const FramedVector<TypeParam::DIMS> rotated_vector =
      a_from_b.rotate(test_vector);

  // Verify identity action is a noop.
  EXPECT_TRUE(rotated_vector.vector().isApprox(test_vector));
  // Verify the return vector frame is correct
  EXPECT_EQ(A, rotated_vector.frame());
}

TYPED_TEST(FramedLieGroupTests, RotateFramedVectorRoundTrip) {
  constexpr double MULTIPLIER = 0.142;
  for (TypeParam a_from_b : make_test_group_elements<TypeParam>()) {
    a_from_b.set_frames(A, B);
    const Eigen::Matrix<double, TypeParam::DIMS, 1> test_vector =
        Eigen::Matrix<double, TypeParam::DIMS, 1>::Ones() * MULTIPLIER;
    const FramedVector<TypeParam::DIMS> test_framed_vector(test_vector, B);
    const FramedVector<TypeParam::DIMS> rotated_vector =
        a_from_b.rotate(test_framed_vector);
    // Transform Back
    const FramedVector<TypeParam::DIMS> recovered_vector =
        a_from_b.inverse().rotate(rotated_vector);

    // Verify the transformed vector frame is correct.
    EXPECT_EQ(A, rotated_vector.frame());
    // Verify the transformed vector is correct.
    EXPECT_TRUE(rotated_vector.isApprox(a_from_b.rotate(test_vector)));
    // Verify the recovered vector frame is correct
    EXPECT_EQ(B, recovered_vector.frame());
    // Verify vector is recovered.
    constexpr double TOL = 10E-10;
    EXPECT_TRUE(recovered_vector.isApprox(test_vector, TOL));
  }
}

TYPED_TEST(FramedLieGroupAssertionTests, FramedVectorTest) {
  // Construct an Identity typed transform.
  const TypeParam a_from_b(TypeParam::identity(A, B));
  const FramedVector<TypeParam::DIMS> bad_test_vector(
      Eigen::Vector3d::Ones(),
      A);
  EXPECT_THROW({ a_from_b.rotate(bad_test_vector); }, AssertException);
}

TYPED_TEST(FramedLieGroupTests, ActionOnVectorTest) {
  // Construct an Identity typed transform.
  const TypeParam a_from_b(TypeParam::identity(A, B));
  const Eigen::Vector3d test_vector = Eigen::Vector3d::Ones();
  const Eigen::Vector3d transformed_vector = a_from_b * test_vector;

  // Verify identity action is a noop.
  EXPECT_TRUE(transformed_vector.isApprox(test_vector));
}

TYPED_TEST(FramedLieGroupTests, InverseTest) {
  // Construct a typed transform from ones.
  const TypeParam a_from_b(
      TypeParam::exp(TypeParam::TangentVector::Ones(), A, B));
  const TypeParam b_from_a = a_from_b.inverse();

  // Verify inverse against group inverse.
  EXPECT_TRUE(a_from_b.inverse().is_approx(b_from_a));
  // Verify that the frames match as expected.
  EXPECT_EQ(B, b_from_a.into());
  EXPECT_EQ(A, b_from_a.from());
}

TYPED_TEST(FramedLieGroupTests, InterpWithGeneratedFrame) {
  // Construct a typed transform from ones.
  const TypeParam a_from_b(
      TypeParam::exp(TypeParam::TangentVector::Ones(), A, B));
  constexpr double HALF = 0.5;
  const TypeParam a_from_c = a_from_b.interp(HALF);

  // Verify the interpolated transform differs from the original.
  EXPECT_FALSE(a_from_b.is_approx(a_from_c));
  // Verify that the frames match as expected.
  EXPECT_EQ(A, a_from_c.into());
  EXPECT_EQ(B, a_from_c.from());
  EXPECT_NE(C, a_from_c.from());
}

TYPED_TEST(FramedLieGroupTests, InterpWithUserFrame) {
  // Construct a typed transform from ones.
  const TypeParam a_from_b(
      TypeParam::exp(TypeParam::TangentVector::Ones(), A, B));
  constexpr double HALF = 0.5;
  const TypeParam a_from_c = a_from_b.interp(HALF, C);

  // Verify the interpolated transform differs from the original.
  EXPECT_FALSE(a_from_b.is_approx(a_from_c));
  // Verify that the frames match as expected.
  EXPECT_EQ(A, a_from_c.into());
  EXPECT_EQ(C, a_from_c.from());
}

TYPED_TEST(FramedLieGroupTests, IsApproxFramedGroup) {
  const TypeParam unframed_a_from_b =
      TypeParam::exp(TypeParam::TangentVector::Ones());
  TypeParam a_from_b = unframed_a_from_b;
  a_from_b.set_frames(A, B);
  const TypeParam &copy_a_from_b(a_from_b);
  TypeParam a_from_c = unframed_a_from_b;
  a_from_c.set_frames(A, C);
  const TypeParam other_a_from_b = TypeParam::identity(A, B);
  EXPECT_TRUE(a_from_b.is_approx(copy_a_from_b));
  EXPECT_FALSE(a_from_b.is_approx(a_from_c));
  EXPECT_FALSE(a_from_b.is_approx(other_a_from_b));
}

TYPED_TEST(FramedLieGroupTests, IsApproxUnframedGroup) {
  const TypeParam unframed_a_from_b =
      TypeParam::exp(TypeParam::TangentVector::Ones());
  const TypeParam unframed_b_from_c = TypeParam::identity();
  TypeParam a_from_b = unframed_a_from_b;
  a_from_b.set_frames(A, B);
  EXPECT_TRUE(a_from_b.is_approx_transform(unframed_a_from_b));
  EXPECT_FALSE(a_from_b.is_approx(unframed_a_from_b));
  EXPECT_FALSE(a_from_b.is_approx(unframed_b_from_c));
}

TYPED_TEST(FramedLieGroupTests, VerifyInto) {
  const TypeParam a_from_b = TypeParam::identity(A, B);
  EXPECT_TRUE(a_from_b.verify_into(A));
  EXPECT_FALSE(a_from_b.verify_into(B));
}

TYPED_TEST(FramedLieGroupTests, VerifyFrom) {
  const TypeParam a_from_b = TypeParam::identity(A, B);
  EXPECT_TRUE(a_from_b.verify_from(B));
  EXPECT_FALSE(a_from_b.verify_from(A));
}

TYPED_TEST(FramedLieGroupTests, VerifyFrames) {
  const TypeParam a_from_b = TypeParam::identity(A, B);
  EXPECT_TRUE(a_from_b.verify_frames(A, B));
  EXPECT_FALSE(a_from_b.verify_frames(A, C));
  EXPECT_FALSE(a_from_b.verify_frames(B, C));
  EXPECT_FALSE(a_from_b.verify_frames(C, C));
}

}  // namespace resim::transforms
