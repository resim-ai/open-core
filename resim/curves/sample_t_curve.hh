// Copyright 2025 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <Eigen/Dense>
#include <vector>

#include "resim/curves/t_curve.hh"
#include "resim/math/multivariate_gaussian.hh"
#include "resim/transforms/se3.hh"
#include "resim/utils/inout.hh"

namespace resim::curves {

// Sample t curves from a given Gaussian distribution. These curves are sampled
// relative to a seed curve. Effectively, we are sampling in the tangent space
// (of the manifold of all TCurves of that size) at the seed_curve.
// @param[in] num_curves - The number of curves to sample.
// @param[in] seed_curve - The seed curve relative to which the samples are
//                         placed.
// @param[in] mean - The mean of the distribution we're sampling. Must have size
//                   optimization::TWO_JET_DOF<SE3>>  *
//                   seed_curve.control_pts().size().
// @param[in] covariance - The covariance of the distribution we're sampling.
//                         Must have size mean.size() x mean.size(). Must be
//                         Positive Semi Definite.
// @returns A vector of num_curves curves sampled from this distribution.
// @throws AssertException if the given covariance matrix is not PSD or sizes
//         are wrong.
std::vector<TCurve<transforms::SE3>> sample_t_curves(
    int num_curves,
    const TCurve<transforms::SE3> &seed_curve,
    Eigen::VectorXd mean,
    Eigen::MatrixXd covariance);

// Overload of the above that works with a pre-existing Gaussian so its solved
// system may be re-used.
// @param[in] num_curves - The number of curves to sample.
// @param[in] seed_curve - The seed curve relative to which the samples are
//                         placed.
// @param[inout] gaussian - A Gaussian which was initialized with mean and cov
//                          matching the requirements in the above overload.
// @returns A vector of num_curves curves sampled from this distribution.
// @throws AssertException if the given covariance matrix is not PSD or sizes
//         are wrong.
std::vector<TCurve<transforms::SE3>> sample_t_curves(
    int num_curves,
    const TCurve<transforms::SE3> &seed_curve,
    InOut<math::Gaussian> gaussian);

}  // namespace resim::curves
