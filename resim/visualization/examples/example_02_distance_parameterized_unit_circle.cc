// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.


#include <cstdlib>
#include <random>

#include "resim/curves/d_curve.hh"
#include "resim/transforms/se3.hh"
#include "resim/transforms/so3.hh"
#include "resim/visualization/view.hh"

using resim::transforms::SE3;  // ReSim's 6 d.o.f. rigid xform.
using resim::transforms::SO3;  // ReSim's 3 d.o.f. rotation.

using Frame = resim::transforms::Frame<3>;  // ReSim's frame identifier.

std::vector<SE3> unit_circle(
    double translation_x = 0.0,
    double translation_y = 0.0,
    double translation_z = 0.0,
    double scale_factor = 1.0) {
  std::vector<SE3> points;

  const SE3 ref_from_000deg(
      {scale_factor * (1 + translation_x),
       scale_factor * translation_y,
       scale_factor * translation_z});
  points.push_back(ref_from_000deg);

  const SE3 ref_from_090deg(
      SO3(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ())),
      {scale_factor * translation_x,
       scale_factor * (1 + translation_y),
       scale_factor * translation_z});
  points.push_back(ref_from_090deg);

  const SE3 ref_from_180deg(
      SO3(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ())),
      {scale_factor * (-1 + translation_x),
       scale_factor * translation_y,
       scale_factor * translation_z});
  points.push_back(ref_from_180deg);

  const SE3 ref_from_270deg(
      SO3(Eigen::AngleAxisd(3 * M_PI / 2, Eigen::Vector3d::UnitZ())),
      {scale_factor * translation_x,
       scale_factor * (-1 + translation_y),
       scale_factor * translation_z});
  points.push_back(ref_from_270deg);

  // Close the circle.
  points.push_back(ref_from_000deg);

  return points;
}

std::vector<SE3> framed_unit_circle(
    const Frame& into_frame,
    const Frame& out_frame,
    double translation_x = 0.0,
    double translation_y = 0.0,
    double translation_z = 0.0,
    double scale_factor = 1.0) {
  // We should construct our unit circle SE3 points first.
  auto raw_se3_points =
      unit_circle(translation_x, translation_y, translation_z, scale_factor);

  // We can add a frame to the SE3 points (into from out).
  std::vector<SE3> points;
  points.reserve(raw_se3_points.size());
  for (auto& point : raw_se3_points) {
    point.set_frames(into_frame, out_frame);
    points.emplace_back(std::move(point));
  }

  return points;
}

int main(int argc, char* argv[]) {
  using resim::transforms::SE3;  // ReSim's 6 d.o.f. rigid xform.
  using resim::transforms::SO3;  // ReSim's 3 d.o.f. rotation.

  // In this example, we will explore creating distance parameterized unit
  // circles using framed transforms.

  // Let's suppose that unit_circle() returns a circle constructed from framed
  // SE3s in the world, from the POV of the robot frame.
  const Frame world = Frame::new_frame();
  const Frame robot = Frame::new_frame();
  // Name the frames:
  VIEW(world) << "world";
  VIEW(robot) << "robot";

  auto world_from_robot_points = framed_unit_circle(world, robot);
  const resim::curves::DCurve world_from_robot_circle(world_from_robot_points);

  // Visualize a control point
  VIEW(*world_from_robot_circle.control_pts().at(0).ref_from_control)
      << "world_from_robot";
  // Visualize the curve
  VIEW(world_from_robot_circle) << "world_from_robot_circle";
  //  Now, suppose we have a robot from sensor transform:
  const Frame sensor = Frame::new_frame();
  VIEW(sensor) << "sensor";

  const double SENSOR_X = 1;
  const double SENSOR_Y = 0.0;
  const double SENSOR_Z = 1;
  const SE3 robot_from_sensor_transform(
      SO3(M_PI_2, {0, 0, 1.0}),
      {SENSOR_X, SENSOR_Y, SENSOR_Z},
      robot,
      sensor);
  VIEW(robot_from_sensor_transform) << "robot_from_sensor";

  // Now, what if we wanted to know what the world from sensor looked like? We
  // should compute the transform using robot_from_sensor_transform.
  std::vector<SE3> world_from_sensor_points;
  world_from_sensor_points.reserve(world_from_robot_points.size());
  for (const auto& world_from_robot : world_from_robot_points) {
    SE3 world_from_sensor_point =
        world_from_robot * robot_from_sensor_transform;
    world_from_sensor_points.push_back(world_from_sensor_point);
  }

  // Plot the resulting circle (sensor to world).
  const resim::curves::DCurve world_from_sensor_circle(
      world_from_sensor_points);
  // Visualize a control point
  VIEW(*world_from_sensor_circle.control_pts().at(0).ref_from_control)
      << "world_from_sensor";
  // Visualize the curve
  VIEW(world_from_sensor_circle) << "transformed_circle";

  return EXIT_SUCCESS;
}
