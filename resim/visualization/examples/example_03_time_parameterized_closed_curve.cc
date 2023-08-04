// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.


#include <Eigen/Dense>
#include <cstdlib>
#include <random>

#include "resim/assert/assert.hh"
#include "resim/curves/d_curve.hh"
#include "resim/testing/random_matrix.hh"
#include "resim/transforms/se3.hh"
#include "resim/transforms/so3.hh"
#include "resim/utils/uuid.hh"
#include "resim/visualization/view.hh"

using resim::transforms::SE3;  // ReSim's 6 d.o.f. rigid xform.
using resim::transforms::SO3;
using TangentVector = SE3::TangentVector;
using TCurve = resim::curves::TCurve<SE3>;  // ReSim's time parameterised curve
using Frame = resim::transforms::Frame<SE3::DIMS>;
using Vec3 = Eigen::Vector3d;
using TwoJetL = resim::curves::TwoJetL<SE3>;

TCurve make_circle_curve(const Frame& into, const Frame& from) {
  constexpr double VELOCITY = 1.0;
  constexpr double ANGULAR_VELOCITY = 1.0;

  const TangentVector velocity{SE3::tangent_vector_from_parts(
      -ANGULAR_VELOCITY * Vec3::UnitZ(),
      -VELOCITY * Vec3::UnitX())};

  std::vector<TCurve::Control> control_points;
  for (double time : {0.0, M_PI_2, M_PI, 3. * M_PI_2, 2. * M_PI}) {
    control_points.push_back({
        .time = time,
        .point =
            TwoJetL{
                SE3{SO3::exp(-(M_PI_2 + time) * Vec3::UnitZ()),
                    Vec3::UnitY(),
                    into,
                    from},
                velocity,
                TangentVector::Zero()},
    });
  }

  return TCurve{control_points};
}

TCurve translate_t_curve(
    const TCurve& curve_from_world,
    const SE3& sensor_from_curve) {
  std::vector<TCurve::Control> control_points;
  for (const TCurve::Control& control : curve_from_world.control_pts()) {
    SE3 curve_from_world = control.point.frame_from_ref();
    SE3 sensor_from_world = sensor_from_curve * curve_from_world;
    resim::view << curve_from_world;
    resim::view << sensor_from_world;
    TCurve::Control new_control{
        control.time,
        TwoJetL{
            sensor_from_world,
            control.point.d_frame_from_ref(),
            control.point.d2_frame_from_ref()},
    };
    control_points.push_back(new_control);
  }
  return TCurve(control_points);
}

int main(int argc, char* argv[]) {
  // In this example, we will explore creating time parameterized unit
  // circles using the ReSim TCurve type.

  const Frame world{resim::UUID{"aaaaaaaa-aaaa-aaaa-aaaa-aaaaaaaaaaaa"}};
  const Frame curve{resim::UUID{"bbbbbbbb-bbbb-bbbb-bbbb-bbbbbbbbbbbb"}};

  VIEW(world) << "world";
  VIEW(curve) << "curve";

  const TCurve circle(make_circle_curve(curve, world));
  VIEW(circle) << "original_circle";

  // Now, let us assume that the curve frame is the frame of a robot, and the
  // circle is the path traced through time. If we know the sensor relative to
  // robot frame, we can transform it into the sensor, to see the path traced by
  // the sensor.

  // Now, suppose we have a robot from sensor transform, which places the sensor
  // w.r.t. the robot, with the same orientation (omitting the rotation):
  const Frame sensor{resim::UUID{"cccccccc-cccc-cccc-cccc-cccccccccccc"}};
  VIEW(sensor) << "sensor";

  const double SENSOR_X = 0.;
  const double SENSOR_Y = 0.;
  const double SENSOR_Z = -1.0;

  const SE3 sensor_from_curve_transform(
      {SENSOR_X, SENSOR_Y, SENSOR_Z},
      sensor,
      curve);

  // We can easily visualize this transform:
  VIEW(sensor_from_curve_transform) << "sensor_from_curve";

  //  We can transform the control points of the TCurve to the sensor frame:
  TCurve sensor_circle = translate_t_curve(circle, sensor_from_curve_transform);
  // and thus visualize the path traced in time for the sensor.
  VIEW(sensor_circle) << "translated_circle";
  return EXIT_SUCCESS;
}
