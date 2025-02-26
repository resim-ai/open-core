
#pragma once

#include <Eigen/Dense>
#include <vector>

#include "resim/curves/t_curve.hh"
#include "resim/transforms/se3.hh"

namespace resim::curves {

std::vector<TCurve<transforms::SE3>> sample_t_curves_pointwise(
    int num_curves,
    const TCurve<transforms::SE3> &seed_curve,
    Eigen::VectorXd point_wise_mean,
    Eigen::MatrixXd point_wise_covariance);

}  // namespace resim::curves
