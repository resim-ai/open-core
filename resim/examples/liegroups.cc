// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

// This file is a companion for the Lie groups documentation at
// https://docs.resim.ai/transforms/using_liegroups/

#include <Eigen/Dense>
#include <cmath>

#include "resim/assert/assert.hh"
#include "resim/transforms/frame.hh"
#include "resim/transforms/se3.hh"
#include "resim/transforms/so3.hh"
#include "resim/visualization/view.hh"

using Vec3 = Eigen::Vector3d;
using resim::transforms::SE3;
using resim::transforms::SO3;
using Frame = resim::transforms::Frame<SE3::DIMS>; /* SE3::DIMS == 3 */

int main(int argc, char **argv) {
  //////////////////////////////////////////////////////////////////////////////
  // Rotations in SO3
  //////////////////////////////////////////////////////////////////////////////

  const double psi = M_PI_4;
  const double theta = 0.5;
  const double phi = 0.1;

  const SO3 scene_from_robot_rotation = SO3(phi, {1., 0., 0.}) *
                                        SO3(theta, {0., 1., 0.}) *
                                        SO3(psi, {0., 0., 1.});

  // Visualize with ReSim View
  // VIEW(scene_from_robot_rotation) << "My rotation";

  const SO3 scene_from_robot_rotation_wrong = SO3::exp({phi, theta, psi});

  // The following assertion will not fail except in the degenerate case of two
  // of the angles being zero.
  REASSERT(
      not scene_from_robot_rotation.is_approx(scene_from_robot_rotation_wrong));

  // Rotating vectors
  const Vec3 robot_forward_in_robot_coordinates{1.0, 0.0, 0.0};
  Vec3 robot_forward_in_scene_coordinates =
      scene_from_robot_rotation * robot_forward_in_robot_coordinates;

  // Can also be more explicit. This is equivalent to the above.
  robot_forward_in_scene_coordinates =
      scene_from_robot_rotation.rotate(robot_forward_in_robot_coordinates);

  // Quaternions
  const Eigen::Quaterniond scene_from_robot_quat{
      scene_from_robot_rotation.quaternion()};
  REASSERT(SO3(scene_from_robot_quat).is_approx(scene_from_robot_rotation));

  // Interpolation
  constexpr double EPSILON = 1e-2;
  const Vec3 axis = Vec3(0.1, 0.2, 0.3).normalized();
  const SO3 scene_from_a(M_PI - EPSILON, axis);
  const SO3 scene_from_b(-M_PI + EPSILON, axis);

  constexpr double FRACTION = 0.5;
  const SO3 scene_from_interped_rotation{
      scene_from_a * (scene_from_a.inverse() * scene_from_b).interp(FRACTION)};

  const SO3 expected(M_PI, axis);
  REASSERT(scene_from_interped_rotation.is_approx(expected));

  //////////////////////////////////////////////////////////////////////////////
  // Poses in SE3
  //////////////////////////////////////////////////////////////////////////////

  const double x = 1.0;
  const double y = 2.0;
  const double z = 3.0;

  const Vec3 scene_from_robot_translation{x, y, z};
  const SE3 scene_from_robot{
      scene_from_robot_rotation,
      scene_from_robot_translation};

  // Transforming points
  const Vec3 point_in_robot_coordinates{Vec3::Random()};
  const Vec3 point_in_scene_coordinates{
      scene_from_robot * point_in_robot_coordinates};
  REASSERT(
      point_in_scene_coordinates ==
      scene_from_robot_rotation * point_in_robot_coordinates +
          scene_from_robot_translation);

  // Transforming vectors
  const Vec3 vector_in_robot_coordinates{Vec3::Random()};
  const Vec3 vector_in_scene_coordinates{
      scene_from_robot.rotate(vector_in_robot_coordinates)};
  REASSERT(
      vector_in_scene_coordinates ==
      scene_from_robot_rotation * vector_in_robot_coordinates);

  // Interpolation
  const SE3 scene_from_interped{scene_from_robot.interp(0.5)};
  REASSERT(
      scene_from_robot.is_approx(scene_from_interped * scene_from_interped));

  //////////////////////////////////////////////////////////////////////////////
  // Tangent Vectors
  //////////////////////////////////////////////////////////////////////////////

  const SE3::TangentVector d_scene_from_robot = SE3::TangentVector::Random();

  const Vec3 robot_angular_velocity_in_robot_coordinates{
      SE3::tangent_vector_rotation_part(d_scene_from_robot)};
  const Vec3 robot_velocity_in_robot_coordinates{
      SE3::tangent_vector_translation_part(d_scene_from_robot)};

  const Vec3 robot_angular_velocity_in_scene_coordinates{
      scene_from_robot.rotation() *
      robot_angular_velocity_in_robot_coordinates};
  (void)robot_angular_velocity_in_scene_coordinates;

  const Vec3 robot_velocity_in_scene_coordinates{
      scene_from_robot.rotation() * robot_velocity_in_robot_coordinates};
  (void)robot_velocity_in_scene_coordinates;

  REASSERT(
      d_scene_from_robot == SE3::tangent_vector_from_parts(
                                robot_angular_velocity_in_robot_coordinates,
                                robot_velocity_in_robot_coordinates));

  //////////////////////////////////////////////////////////////////////////////
  // Exp and Log
  //////////////////////////////////////////////////////////////////////////////

  const SE3::TangentVector my_tangent_vector{SE3::TangentVector::Random()};
  REASSERT(my_tangent_vector.isApprox(SE3::exp(my_tangent_vector).log()));

  //////////////////////////////////////////////////////////////////////////////
  // Exp and Log
  //////////////////////////////////////////////////////////////////////////////
  const Frame world{Frame::new_frame()};
  const Frame robot{Frame::new_frame()};
  const Frame sensor{Frame::new_frame()};

  // The pose of the robot in the world
  const SE3 world_from_robot{SO3::identity(), {5., 5., 0.}, world, robot};

  // VIEW(world) << "World frame";
  // VIEW(robot) << "Robot frame";
  // VIEW(world_from_robot) << "World from robot";

  // The pose of a sensor mounted on the robot
  const SE3 robot_from_sensor{
      SO3{M_PI_2, {0., 0., 1.}},
      {0., 0., 1.},
      robot,
      sensor};

  // VIEW(sensor) << "Sensor frame";
  // VIEW(robot_from_sensor) << "Robot from sensor";

  const SE3 world_from_sensor{world_from_robot * robot_from_sensor};
  REASSERT(world_from_sensor.is_framed());
  REASSERT(world_from_sensor.into() == world);
  REASSERT(world_from_sensor.from() == sensor);

  // Whoops! This fails at run time because we forgot to invert
  // robot_from_sensor!
  // const SE3 robot_from_world{world_from_sensor * robot_from_sensor};

  // We should have done:
  const SE3 robot_from_world{world_from_sensor * robot_from_sensor.inverse()};

  return EXIT_SUCCESS;
}
