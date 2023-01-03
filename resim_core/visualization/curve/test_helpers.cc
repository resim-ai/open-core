
#include "resim_core/visualization/curve/test_helpers.hh"

#include <Eigen/Dense>
#include <cmath>

#include "resim_core/transforms/se3.hh"
#include "resim_core/transforms/so3.hh"

namespace resim::visualization::curve::testing {
using transforms::FSE3;
using transforms::SE3;
using transforms::SO3;
using TangentVector = FSE3::TangentVector;
using Vec3 = Eigen::Vector3d;
using TwoJet = curves::TwoJet<FSE3>;

curves::TCurve<transforms::FSE3> make_circle_curve(
    const transforms::Frame<3> &into,
    const transforms::Frame<3> &from) {
  constexpr double VELOCITY = 1.0;
  constexpr double ANGULAR_VELOCITY = 1.0;

  const TangentVector velocity{FSE3::tangent_vector_from_parts(
      -ANGULAR_VELOCITY * Vec3::UnitZ(),
      -VELOCITY * Vec3::UnitX())};

  std::vector<curves::TCurve<transforms::FSE3>::Control> control_points;
  for (double time : {0.0, M_PI_2, M_PI, 3. * M_PI_2, 2. * M_PI}) {
    control_points.push_back({
        .time = time,
        .point =
            TwoJet{
                FSE3{
                    SE3{SO3::exp(-(M_PI_2 + time) * Vec3::UnitZ()),
                        Vec3::UnitY()},
                    into,
                    from},
                velocity,
                TangentVector::Zero()},
    });
  }

  return curves::TCurve<transforms::FSE3>{control_points};
}

}  // namespace resim::visualization::curve::testing
