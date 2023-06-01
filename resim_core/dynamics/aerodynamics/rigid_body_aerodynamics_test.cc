#include "resim_core/dynamics/aerodynamics/rigid_body_aerodynamics.hh"

#include <gtest/gtest.h>

#include <cmath>
#include <functional>
#include <memory>
#include <vector>

#include "resim_core/assert/assert.hh"

namespace resim::dynamics::aerodynamics {

namespace {
using transforms::SE3;
using transforms::SO3;
using Frame = transforms::Frame<SE3::DIMS>;
using RigidBodyState = actor::state::RigidBodyState<SE3>;
const Frame COM_FRAME = Frame::new_frame();
const Frame COP_FRAME = Frame::new_frame();
const Frame REF_FRAME = Frame::new_frame();
using FramedVector = transforms::FramedVector<SE3::DIMS>;
constexpr double HALF = 0.5;
constexpr double BLADE_CANT_ANGLE = M_PI_4 / 2;
constexpr double BLADE_COP_RADIUS = 2.5;
constexpr double RHO = 1.2;        // Atmospheric density (kg/m^2).
constexpr double BLADE_AREA = 5.;  // In m^2, based on a 5x1 blade.

struct EmptyElementState : public AerodynamicElementState {
  EmptyElementState() = default;
};
class DummyAerodynamicElement : public AerodynamicElementImpl<
                                    DummyAerodynamicElement,
                                    EmptyElementState> {
  using AerodynamicElementImpl::AerodynamicElementImpl;

 protected:
  static FramedVector aerodynamics_impl_(
      const FramedVector &cop_local_wind,
      const EmptyElementState &state) {
    return FramedVector(Eigen::Vector3d::Zero(), cop_local_wind.frame());
  }

  friend class AerodynamicElementImpl<
      DummyAerodynamicElement,
      EmptyElementState>;
};

// Returns an estimate of the lift force on a toy windmill blade.
// Drag forces are neglected completely.
const auto windmill_blade = [](const FramedVector &cop_local_wind,
                               const double blade_area) {
  // Blade is aligned with the X-Y plane in the COP frame.
  const Eigen::Vector3d wind_dir = cop_local_wind.vector().normalized();
  const Eigen::Vector3d in_plane_wind_dir =
      Eigen::Vector3d({wind_dir.x(), wind_dir.y(), 0.}).normalized();
  const Eigen::Vector3d in_plane_wind_orth = wind_dir.cross(in_plane_wind_dir);
  // If we approximate the coefficient of lift at sin(angle_of_attack) then:
  const double lift_coeff = in_plane_wind_orth.norm();
  // Calculate the force magnitude.
  const double speed = cop_local_wind.norm();
  const double lift_force_mag =
      HALF * RHO * speed * speed * lift_coeff * blade_area;
  // Calculate the force vector.
  const Eigen::Vector3d force_dir =
      wind_dir.cross(in_plane_wind_orth).normalized();
  // Force framed vector.
  return FramedVector(lift_force_mag * force_dir, cop_local_wind.frame());
};

struct WindmillElementState : public AerodynamicElementState {
  const double &blade_area = BLADE_AREA;

  WindmillElementState() = default;
  explicit WindmillElementState(const double &area) : blade_area(area){};
};

class WindmillAerodynamicElement : public AerodynamicElementImpl<
                                       WindmillAerodynamicElement,
                                       WindmillElementState> {
  using AerodynamicElementImpl::AerodynamicElementImpl;

 protected:
  static FramedVector aerodynamics_impl_(
      const FramedVector &cop_local_wind,
      const WindmillElementState &state) {
    return windmill_blade(cop_local_wind, state.blade_area);
  }

  friend class AerodynamicElementImpl<
      WindmillAerodynamicElement,
      WindmillElementState>;
};

RigidBodyAerodynamics build_windmill() {
  // Create separate COP frames for each blade.
  Frame blade_1_cop_frame = Frame::new_frame();
  Frame blade_2_cop_frame = Frame::new_frame();
  // Make the blade transforms.
  const std::shared_ptr<AerodynamicElement> blade_1 =
      std::make_shared<WindmillAerodynamicElement>(SE3(
          SO3(Eigen::AngleAxisd(-BLADE_CANT_ANGLE, Eigen::Vector3d::UnitY())),
          {0., -BLADE_COP_RADIUS, 0.},
          COM_FRAME,
          blade_1_cop_frame));
  const std::shared_ptr<AerodynamicElement> blade_2 =
      std::make_shared<WindmillAerodynamicElement>(SE3(
          SO3(Eigen::AngleAxisd(BLADE_CANT_ANGLE, Eigen::Vector3d::UnitY())),
          {0., BLADE_COP_RADIUS, 0.},
          COM_FRAME,
          blade_2_cop_frame));
  // Add blades to a RigidBodyAerodynamics object.
  RigidBodyAerodynamics windmill{blade_1, blade_2};
  return windmill;
}

const WindmillElementState blade1_state{};
const WindmillElementState blade2_state{};
const std::vector<std::reference_wrapper<const AerodynamicElementState>>
    windmill_states{blade1_state, blade2_state};

}  // namespace

TEST(AerodynamicElementTests, ConstructStates) {
  const double area = 1.2;

  const WindmillElementState windmill_state{area};
  EXPECT_EQ(windmill_state.blade_area, area);

  const WindmillElementState default_windmill_state{};
  EXPECT_EQ(default_windmill_state.blade_area, BLADE_AREA);

  const std::unique_ptr<WindmillElementState> state_ptr =
      std::make_unique<WindmillElementState>();

  EXPECT_EQ(state_ptr->blade_area, BLADE_AREA);
}
TEST(AerodynamicElementTests, ConstructAerodynamicElement) {
  const SE3 identity = SE3::identity(COM_FRAME, COP_FRAME);
  const DummyAerodynamicElement test_element(identity);
  EXPECT_TRUE(test_element.com_from_cop().is_approx(identity));
  EXPECT_EQ(test_element.com_frame(), COM_FRAME);
  EXPECT_EQ(test_element.cop_frame(), COP_FRAME);
}

TEST(AerodynamicElementAssertionTests, BadFrames) {
  const Frame BAD_FRAME = Frame::new_frame();
  const SE3 identity = SE3::identity(COM_FRAME, COP_FRAME);
  const DummyAerodynamicElement test_element(identity);
  // Create a body with correct frames
  RigidBodyState good_state(SE3::identity(REF_FRAME, COM_FRAME));
  // Create a body with incorrect frames
  RigidBodyState bad_state(SE3::identity(REF_FRAME, BAD_FRAME));
  FramedVector good_wind(Eigen::Vector3d::Zero(), REF_FRAME);
  FramedVector bad_wind(Eigen::Vector3d::Zero(), BAD_FRAME);
  EXPECT_THROW(
      {
        auto local_wind = test_element.cop_local_wind(bad_state, good_wind);
        (void)local_wind;
      },
      AssertException);
  EXPECT_THROW(
      {
        auto local_wind = test_element.cop_local_wind(good_state, bad_wind);
        (void)local_wind;
      },
      AssertException);
}
TEST(AerodynamicElementTest, PureRotationTest) {
  // Create a body aligned with the reference frame.
  RigidBodyState test_state(SE3::identity(REF_FRAME, COM_FRAME));
  ASSERT_EQ(test_state.ref_from_body().into(), REF_FRAME);
  ASSERT_EQ(test_state.ref_from_body().from(), COM_FRAME);
  ASSERT_TRUE(test_state.body_angular_velocity_radps().isZero());
  ASSERT_TRUE(test_state.body_linear_velocity_mps().isZero());
  // Apply an angular velocity of 1 rad/s about the X-axis;
  test_state.set_body_angular_velocity_radps(Eigen::Vector3d::UnitX());
  ASSERT_FALSE(test_state.body_angular_velocity_radps().isZero());
  // Create an aerodynamic element with centre of pressure displaced one meter
  // in the Y-axis.
  DummyAerodynamicElement test_element(
      SE3(Eigen::Vector3d::UnitY(), COM_FRAME, COP_FRAME));
  // Zero ambient wind
  const FramedVector ref_local_wind(Eigen::Vector3d::Zero(), REF_FRAME);
  // Expected COP local wind.
  // With no ambient wind the apparent local wind experienced by the COP is
  // due only to its motion. As the body is in pure rotation the velocity of
  // the COP is given by r*omega (1*1). Therefore we expect a velocity of 1 in
  // the Z (up) direction and a corresponding wind -1 in the Z direction.
  const FramedVector expected_cop_local_wind(
      -Eigen::Vector3d::UnitZ(),
      COP_FRAME);

  // VERIFICATION
  EXPECT_TRUE(test_element.cop_local_wind(test_state, ref_local_wind)
                  .isApprox(expected_cop_local_wind));

  // Because the wind is in the COP frame we expect it to be independent of the
  // rotational pose of the rotation body. For example:
  const SO3 ref_from_body_rot(M_PI_2, Eigen::Vector3d::UnitX());
  test_state.set_ref_from_body(SE3(ref_from_body_rot, REF_FRAME, COM_FRAME));

  // Repeat the same verification
  EXPECT_TRUE(test_element.cop_local_wind(test_state, ref_local_wind)
                  .isApprox(expected_cop_local_wind));
}

TEST(AerodynamicElementTest, HelicalMotionTest) {
  // Create a body aligned with the reference frame.
  RigidBodyState test_state(SE3::identity(REF_FRAME, COM_FRAME));
  // Apply an angular velocity of 1 rad/s about the X-axis;
  test_state.set_body_angular_velocity_radps(Eigen::Vector3d::UnitX());
  // Apply a (forward) linear velocity of 1 m/s in the X-axis direction;
  test_state.set_body_linear_velocity_mps(Eigen::Vector3d::UnitX());
  // Create an aerodynamic element with centre of pressure displaced one meter
  // in the Y-axis.
  DummyAerodynamicElement test_element(
      SE3(Eigen::Vector3d::UnitY(), COM_FRAME, COP_FRAME));
  // Zero ambient wind
  FramedVector ref_local_wind(Eigen::Vector3d::Zero(), REF_FRAME);
  // Expected COP local wind.
  // With no ambient wind the apparent local wind experienced by the COP is
  // due only to its motion. As the body is rotating the velocity of
  // the COP due to rotation is given by r*omega (1*1) and is in the Z (up)
  // direction. The body is also moving forwards (in X) with velocity 1m/s
  // Therefore the expected resultant velocity vector is {1., 0., 1.}. The
  // COP local wind vector is the opposite vector {-1, 0., -1}.
  const FramedVector expected_cop_local_wind({-1., 0., -1}, COP_FRAME);

  // VERIFICATION
  EXPECT_TRUE(test_element.cop_local_wind(test_state, ref_local_wind)
                  .isApprox(expected_cop_local_wind));

  // A forward velocity should be indistinguishable from a head-wind of the same
  // magnitude. Set velocity to 0.
  test_state.set_body_linear_velocity_mps(Eigen::Vector3d::Zero());
  // Create a 1m/s headwind.
  ref_local_wind = FramedVector(-Eigen::Vector3d::UnitX(), REF_FRAME);

  // Repeat the same verification
  EXPECT_TRUE(test_element.cop_local_wind(test_state, ref_local_wind)
                  .isApprox(expected_cop_local_wind))
      << test_element.cop_local_wind(test_state, ref_local_wind).vector();

  // Because the wind is in the COP frame we expect it to be independent of the
  // rotational pose of the rotation body. For example:
  const SO3 ref_from_body_rot(M_PI_2, Eigen::Vector3d::UnitX());
  test_state.set_ref_from_body(SE3(ref_from_body_rot, REF_FRAME, COM_FRAME));

  // Repeat the same verification
  EXPECT_TRUE(test_element.cop_local_wind(test_state, ref_local_wind)
                  .isApprox(expected_cop_local_wind));
}

TEST(WindmillBladeTest, ZeroAngleOfAttack) {
  // Test wind vectors in the X-Y plane -(all should yield zero force).
  FramedVector wind(Eigen::Vector3d::UnitX(), COP_FRAME);
  const SO3 current_from_incremented(
      Eigen::AngleAxisd(M_PI_4, Eigen::Vector3d::UnitZ()),
      COP_FRAME,
      COP_FRAME);
  constexpr unsigned VECTOR_COUNT = 8;
  for (unsigned int i = 0; i < VECTOR_COUNT; ++i) {
    FramedVector pressure_force = windmill_blade(wind, BLADE_AREA);
    EXPECT_DOUBLE_EQ(pressure_force.norm(), 0.);
    wind = current_from_incremented.rotate(wind);
  }
}

TEST(WindmillBladeTest, PositiveAngleOfAttack) {
  // Test wind vectors with a positive Z component, that is, blowing the
  // underside of the blade.
  FramedVector wind({1., 0., HALF}, COP_FRAME);
  const SO3 current_from_incremented(
      Eigen::AngleAxisd(M_PI_4, Eigen::Vector3d::UnitZ()),
      COP_FRAME,
      COP_FRAME);
  double last_force_cos = 0.;
  constexpr unsigned VECTOR_COUNT = 8;
  for (unsigned int i = 0; i < VECTOR_COUNT; ++i) {
    FramedVector pressure_force = windmill_blade(wind, BLADE_AREA);
    // Force angle to Z-axis should be the same for all wind vectors
    double force_cos =
        pressure_force.normalized().dot(Eigen::Vector3d::UnitZ());
    if (last_force_cos > 0.) {
      EXPECT_DOUBLE_EQ(force_cos, last_force_cos);
    }
    last_force_cos = force_cos;
    // Force should be non-zero.
    EXPECT_GT(pressure_force.norm(), 0.);
    // Force should be generally up.
    EXPECT_GT(pressure_force.z(), 0.);
    wind = current_from_incremented.rotate(wind);
  }
}

TEST(WindmillBladeTest, NegativeAngleOfAttack) {
  // Test Wind vectors with a negative Z component, that is, blowing atop the
  // blade.
  FramedVector wind({1., 0., -HALF}, COP_FRAME);
  const SO3 current_from_incremented(
      Eigen::AngleAxisd(M_PI_4, Eigen::Vector3d::UnitZ()),
      COP_FRAME,
      COP_FRAME);
  double last_force_cos = 0.;
  constexpr unsigned VECTOR_COUNT = 8;
  for (unsigned int i = 0; i < VECTOR_COUNT; ++i) {
    FramedVector pressure_force = windmill_blade(wind, BLADE_AREA);
    // Force angle to Z-axis should be the same for all wind vectors
    double force_cos =
        pressure_force.normalized().dot(Eigen::Vector3d::UnitZ());
    if (last_force_cos > 0.) {
      EXPECT_DOUBLE_EQ(force_cos, last_force_cos);
    }
    last_force_cos = force_cos;
    // Force should be non-zero.
    EXPECT_GT(pressure_force.norm(), 0.);
    // Force should be generally down.
    EXPECT_LT(pressure_force.z(), 0.);
    wind = current_from_incremented.rotate(wind);
  }
}

TEST(RigidBodyAerodynamicsTest, ConstructionFromVector) {
  const SE3 com_from_cop = SE3::identity(COM_FRAME, COP_FRAME);
  std::vector<std::shared_ptr<AerodynamicElement>> components{
      std::make_shared<DummyAerodynamicElement>(com_from_cop),
      std::make_shared<DummyAerodynamicElement>(com_from_cop),
      std::make_shared<DummyAerodynamicElement>(com_from_cop),
  };
  const unsigned component_count = components.size();
  RigidBodyAerodynamics body(std::move(components));
  EXPECT_EQ(body.components().size(), component_count);
}

TEST(RigidBodyAerodynamicsAssertionTest, ConstructionWithBadFrames) {
  const SE3 com_from_cop = SE3::identity(COM_FRAME, COP_FRAME);
  const SE3 bad_transform = SE3::identity(COP_FRAME, COM_FRAME);
  RigidBodyAerodynamics body{
      std::make_shared<DummyAerodynamicElement>(com_from_cop)};
  EXPECT_THROW(
      {
        body.append(std::make_shared<DummyAerodynamicElement>(bad_transform));
      },
      AssertException);
}

TEST(RigidBodyAerodynamicsTest, StaticWindmill) {
  // Create a body aligned with the reference frame.
  RigidBodyState windmill_state(SE3::identity(REF_FRAME, COM_FRAME));

  // Create a 1m/s headwind.
  FramedVector ref_local_wind(-Eigen::Vector3d::UnitX(), REF_FRAME);
  // Build the windmill
  RigidBodyAerodynamics windmill = build_windmill();

  // Check the two blades individually.
  ASSERT_EQ(windmill.components().size(), 2);
  const std::shared_ptr<AerodynamicElement> &blade_1 =
      windmill.components().at(0);
  const std::shared_ptr<AerodynamicElement> &blade_2 =
      windmill.components().at(1);

  const FramedVector blade_1_force = blade_1->cop_local_force(
      windmill_state,
      ref_local_wind,
      windmill_states.at(0));
  const FramedVector blade_2_force = blade_2->cop_local_force(
      windmill_state,
      ref_local_wind,
      windmill_states.at(1));

  // Force should be non-zero
  EXPECT_GT(blade_1_force.norm(), 0.);
  // Both blades should have equal force magnitude.
  EXPECT_DOUBLE_EQ(blade_1_force.norm(), blade_2_force.norm());
  // Z component of forces should cancel.
  EXPECT_DOUBLE_EQ((blade_1_force.z() + blade_2_force.z()), 0.);

  typename SE3::TangentVector pressure_force = windmill.body_pressure_force(
      windmill_state,
      ref_local_wind,
      windmill_states);

  // VERIFICATION
  // We expect there to be no net force on the windmill and only a torque
  // about the windmill body-X axis.
  constexpr size_t NUM_ZEROS = 5;
  EXPECT_TRUE(pressure_force.tail(NUM_ZEROS).isZero());
  // We expect clockwise rotation about body-X, hence a negative torque.
  EXPECT_LT(pressure_force[0], 0.);
}

TEST(RigidBodyAerodynamicsTest, RotatingWindmill) {
  // Create a body aligned with the reference frame.
  RigidBodyState windmill_state(SE3::identity(REF_FRAME, COM_FRAME));
  // Create a 1m/s headwind.
  FramedVector ref_local_wind(-Eigen::Vector3d::UnitX(), REF_FRAME);
  // Build the windmill
  RigidBodyAerodynamics windmill = build_windmill();
  // Rotate the windmill at theoretical max speed.a
  // As the windmill spins, the motion of the blade induces an apparent wind,
  // which reduces the angle of attack. Thus, for a given headwind, the windmill
  // has a theoretical maximum angular speed, where angle-of-attack goes to
  // 0. For a headwind of 1m/s, that max speed is given by:
  const double max_omega = tan(BLADE_CANT_ANGLE) / BLADE_COP_RADIUS;
  // Apply the max speed to the windmill in the clockwise direction.
  windmill_state.set_body_angular_velocity_radps({-max_omega, 0., 0.});
  // Obtain the new pressure force at this speed
  typename SE3::TangentVector pressure_force = windmill.body_pressure_force(
      windmill_state,
      ref_local_wind,
      windmill_states);
  // verify all torques and forces are zero.
  EXPECT_TRUE(pressure_force.isZero());
}

TEST(RigidBodyAerodynamicsTest, BadDynamicCast) {
  RigidBodyState windmill_state(SE3::identity(REF_FRAME, COM_FRAME));
  FramedVector ref_local_wind(-Eigen::Vector3d::UnitX(), REF_FRAME);
  // Build the windmill
  RigidBodyAerodynamics windmill = build_windmill();

  // Add state for wrong element type
  const EmptyElementState s1{};
  const WindmillElementState s2{};

  std::vector<std::reference_wrapper<const AerodynamicElementState>> states{
      s1,
      s2};
  // Expect bad cast
  EXPECT_THROW(
      windmill.body_pressure_force(windmill_state, ref_local_wind, states),
      AssertException);
}

TEST(RigidBodyAerodynamicsTest, BadLength) {
  RigidBodyState windmill_state(SE3::identity(REF_FRAME, COM_FRAME));
  FramedVector ref_local_wind(-Eigen::Vector3d::UnitX(), REF_FRAME);
  // Build the windmill
  RigidBodyAerodynamics windmill = build_windmill();

  const WindmillElementState s1{};
  const WindmillElementState s2{};
  const WindmillElementState s3{};

  std::vector<std::reference_wrapper<const AerodynamicElementState>>
      too_many_states{s1, s2, s3};

  // Expect assertion errors
  EXPECT_THROW(
      windmill
          .body_pressure_force(windmill_state, ref_local_wind, too_many_states),
      AssertException);

  std::vector<std::reference_wrapper<const AerodynamicElementState>>
      not_enough_states{s1};
  EXPECT_THROW(
      windmill.body_pressure_force(
          windmill_state,
          ref_local_wind,
          not_enough_states),
      AssertException);
}

}  // namespace resim::dynamics::aerodynamics
