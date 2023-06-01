
#include "resim_core/curves/test_helpers.hh"

#include <Eigen/Dense>
#include <cmath>

#include "resim_core/transforms/se3.hh"
#include "resim_core/transforms/so3.hh"

namespace resim::curves::testing {
using transforms::SE3;
using transforms::SO3;
using TangentVector = SE3::TangentVector;
using Vec3 = Eigen::Vector3d;
using TwoJetL = curves::TwoJetL<SE3>;

curves::TCurve<transforms::SE3> make_circle_curve(
    const transforms::Frame<3> &into,
    const transforms::Frame<3> &from) {
  constexpr double VELOCITY = 1.0;
  constexpr double ANGULAR_VELOCITY = 1.0;

  const TangentVector velocity{SE3::tangent_vector_from_parts(
      -ANGULAR_VELOCITY * Vec3::UnitZ(),
      -VELOCITY * Vec3::UnitX())};

  std::vector<curves::TCurve<transforms::SE3>::Control> control_points;
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

  return curves::TCurve<transforms::SE3>{control_points};
}

}  // namespace resim::curves::testing
