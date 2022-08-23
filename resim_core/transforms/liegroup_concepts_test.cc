//
// liegroup_concepts_test.cc
//
// This test suite tests type traits for identifying Lie Groups. Notably, these
// tests are testing type traits with static asserts, so they actually do
// nothing when executed. Basically, these tests pass if they compile.
//
#include "resim_core/transforms/liegroup_concepts.hh"

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <string>

#include "resim_core/transforms/framed_group.hh"
#include "resim_core/transforms/se3.hh"
#include "resim_core/transforms/so3.hh"

namespace resim::transforms {

// Test that we can correctly identify objects with the correct composition
// operator.
TEST(LieGroupTraitsTest, TestHasComposition) {
  // SETUP
  struct HasComposition {
    HasComposition operator*(const HasComposition &) const;
  };
  struct HasCompositionWrongSignatures {
    HasCompositionWrongSignatures operator*(
        const HasCompositionWrongSignatures &);
    HasCompositionWrongSignatures operator*(
        HasCompositionWrongSignatures &) const;
    HasCompositionWrongSignatures operator*(
        HasCompositionWrongSignatures &&) const;
    int operator*(const HasCompositionWrongSignatures &) const;
  };
  struct DoesntHaveComposition {};

  // ACTION / VERIFICATION
  static_assert(has_composition<HasComposition>);
  static_assert(!has_composition<HasCompositionWrongSignatures>);
  static_assert(!has_composition<DoesntHaveComposition>);
}

// Test that we can correctly identify objects with the correct identity member
// function.
TEST(LieGroupTraitsTest, TestHasIdentity) {
  // SETUP
  struct HasIdentity {
    static HasIdentity identity();
  };
  struct HasIdentityWrongSignature {
    static int identity();
  };
  struct ObjectWithoutIdentity {};

  // ACTION / VERIFICATION
  static_assert(has_identity<HasIdentity>);
  static_assert(!has_identity<HasIdentityWrongSignature>);
  static_assert(!has_identity<ObjectWithoutIdentity>);
}

// Test that we can correctly identify objects with the correct inverse member
// function.
TEST(LieGroupTraitsTest, TestHasInverse) {
  // SETUP
  struct HasInverse {
    HasInverse inverse() const;
  };
  struct HasNonConstInverse {
    HasNonConstInverse inverse();
  };
  struct DoesntHaveInverse {};

  // ACTION / VERIFICATION
  static_assert(has_inverse<HasInverse>);
  static_assert(!has_inverse<HasNonConstInverse>);
  static_assert(!has_inverse<DoesntHaveInverse>);
}

// Test that we can correctly identify objects that inherit LieGroup.
TEST(LieGroupTraitsTest, TestInheritsLieGroup) {
  // SETUP
  struct InheritsLieGroup : public LieGroup<2, 1> {};
  struct InheritsLieGroupPrivate : private LieGroup<2, 1> {};
  struct DoesntInheritLieGroup {};

  // ACTION / VERIFICATION
  static_assert(inherits_liegroup<InheritsLieGroup>);
  static_assert(!inherits_liegroup<InheritsLieGroupPrivate>);
  static_assert(!inherits_liegroup<DoesntInheritLieGroup>);
}

// Test that we can correctly identify objects that have a Group action.
TEST(LieGroupTraitsTest, TestHasAction) {
  // SETUP
  struct HasAction : public LieGroup<2, 1> {
    Eigen::Vector2d operator*(const Eigen::Vector2d &) const;
  };
  struct HasActionWrongSignatures : public LieGroup<2, 1> {
    Eigen::Vector2d operator*(Eigen::Vector2d &) const;
    Eigen::Vector2d operator*(Eigen::Vector2d &&) const;
    Eigen::Vector2d operator*(const Eigen::Vector2d &);
    Eigen::Vector2d operator*(Eigen::Vector2d &);
    Eigen::Vector2d operator*(Eigen::Vector2d &&);
    Eigen::Vector2d operator*(const Eigen::Vector2d &&);
    int operator*(const Eigen::Vector2d &) const;
  };
  struct DoesntHaveAction {};

  // ACTION / VERIFICATION
  static_assert(has_action<HasAction>);
  static_assert(!has_action<HasActionWrongSignatures>);
  static_assert(!has_action<DoesntHaveAction>);
}

// Test that we can correctly identify objects that have an exponential member
// function
TEST(LieGroupTraitsTest, TestHasExp) {
  // SETUP
  struct HasExp : public LieGroup<2, 1> {
    static HasExp exp(const TangentVector &);
  };
  struct HasExpWrongSignatures : public LieGroup<2, 1> {
    static HasExpWrongSignatures exp(TangentVector &);
    static HasExpWrongSignatures exp(TangentVector &&);
    static int exp(const TangentVector &);
  };
  struct DoesntHaveExp {};

  // ACTION / VERIFICATION
  static_assert(has_exp<HasExp>);
  static_assert(!has_exp<HasExpWrongSignatures>);
  static_assert(!has_exp<DoesntHaveExp>);
}

// Test that we can correctly identify objects that have a logarithm member
// function.
TEST(LieGroupTraitsTest, TestHasLog) {
  // SETUP
  struct HasLog : public LieGroup<2, 1> {
    TangentVector log() const;
  };
  struct HasLogWrongSignatures : public LieGroup<2, 1> {
    TangentVector log();
    int log() const;
  };
  struct DoesntHaveLog {};

  // ACTION / VERIFICATION
  static_assert(has_log<HasLog>);
  static_assert(!has_log<HasLogWrongSignatures>);
  static_assert(!has_log<DoesntHaveLog>);
}

// Test that we can correctly identify types that have all of the right adjoint
// member functions.
TEST(LieGroupTraitsTest, TestHasAdjoint) {
  // SETUP
  struct HasAdjoint : public LieGroup<2, 1> {
    TangentMapping adjoint() const;
    static TangentMapping adjoint(const TangentVector &);
    TangentVector adjoint_times(const TangentVector &) const;
  };
  struct HasAdjointWrongSignatures : public LieGroup<2, 1> {
    TangentMapping adjoint();
    int adjoint() const;

    static TangentMapping adjoint(TangentVector &);
    static TangentMapping adjoint(TangentVector &&);
    static int adjoint(const TangentVector &);

    TangentVector adjoint_times(TangentVector &) const;
    TangentVector adjoint_times(TangentVector &&) const;
    TangentVector adjoint_times(const TangentVector &);
    int adjoint_times(const TangentVector &) const;
  };
  struct DoesntHaveAdjoint {};

  // ACTION / VERIFICATION
  static_assert(has_adjoint<HasAdjoint>);
  static_assert(!has_adjoint<HasAdjointWrongSignatures>);
  static_assert(!has_adjoint<DoesntHaveAdjoint>);

  static_assert(has_adjoint_times<HasAdjoint>);
  static_assert(!has_adjoint_times<HasAdjointWrongSignatures>);
  static_assert(!has_adjoint_times<DoesntHaveAdjoint>);

  static_assert(has_algebra_adjoint<HasAdjoint>);
  static_assert(!has_algebra_adjoint<HasAdjointWrongSignatures>);
  static_assert(!has_algebra_adjoint<DoesntHaveAdjoint>);
}

// Test that we can correctly identify types with the right is_approx member
// function.
TEST(LieGroupTraitsTest, TestHasIsApprox) {
  // SETUP
  struct HasIsApprox : public LieGroup<2, 1> {
    bool is_approx(const HasIsApprox &) const;
  };
  struct HasIsApproxWrongSignatures : public LieGroup<2, 1> {
    bool is_approx(const HasIsApprox &);
    bool is_approx(HasIsApprox &) const;
    bool is_approx(HasIsApprox &&) const;
    bool is_approx(const HasIsApprox &) const;
  };
  struct DoesntHaveApprox {};

  // ACTION / VERIFICATION
  static_assert(has_is_approx<HasIsApprox>);
  static_assert(!has_is_approx<HasIsApproxWrongSignatures>);
  static_assert(!has_is_approx<DoesntHaveApprox>);
}

// Test that we can identify types with the right interp member function().
TEST(LieGroupTraitsTest, TestHasInterp) {
  // SETUP
  struct HasInterp : public LieGroup<2, 1> {
    HasInterp interp(double) const;
  };
  struct HasInterpWrongSignatures : public LieGroup<2, 1> {
    HasInterpWrongSignatures interp(double);
    double interp(double) const;
  };
  struct DoesntHaveInterp {};

  // ACTION / VERIFICATION
  static_assert(has_interp<HasInterp>);
  static_assert(!has_interp<HasInterpWrongSignatures>);
  static_assert(!has_interp<DoesntHaveInterp>);
}

// Test that we can correctly identify Lie groups.
TEST(LieGroupTraitsTest, TestIsLieGroup) {
  // SETUP / ACTION / VERIFICATION
  static_assert(LieGroupType<SE3>);
  static_assert(LieGroupType<SO3>);
  static_assert(LieGroupType<FSE3>);
  static_assert(LieGroupType<FSO3>);
  static_assert(!LieGroupType<int>);
  static_assert(!LieGroupType<std::string>);
}

}  // namespace resim::transforms
