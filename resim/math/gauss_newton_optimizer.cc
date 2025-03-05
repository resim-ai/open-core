// Copyright 2025 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/math/gauss_newton_optimizer.hh"

#include <Eigen/Sparse>
#include <limits>
#include <utility>

namespace resim::math {

int GaussNewtonOptimizer::parameters_offset(const std::string &key) const {
  REASSERT(parameter_block_offsets_.contains(key), "Parameter key not found!");
  return parameter_block_offsets_.at(key);
}

int GaussNewtonOptimizer::parameters_dof(const std::string &key) const {
  REASSERT(parameters_.current().contains(key), "Parameter key not found!");
  const auto &param_block_ptr = parameters_.current().at(key);
  REASSERT(param_block_ptr != nullptr, "Invalid parameter block!");
  return param_block_ptr->dof();
}

void GaussNewtonOptimizer::register_error_model(
    const std::string &key,
    std::unique_ptr<ErrorModel> error_model) {
  REASSERT(error_model != nullptr, "Invalid error model cannot be registered!");
  REASSERT(
      error_block_offsets_.emplace(key, errors_dof_).second,
      "Duplicate error key!");
  errors_dof_ += error_model->dof();
  REASSERT(
      models_.emplace(key, std::move(error_model)).second,
      "Duplicate error key!");
}

int GaussNewtonOptimizer::errors_offset(const std::string &key) const {
  REASSERT(error_block_offsets_.contains(key), "Model key not found!");
  return error_block_offsets_.at(key);
}

int GaussNewtonOptimizer::errors_dof(const std::string &key) const {
  REASSERT(models_.contains(key), "Model key not found!");
  const auto &model_ptr = models_.at(key);
  REASSERT(model_ptr != nullptr, "Invalid model!");
  return model_ptr->dof();
}

GaussNewtonOptimizer::Result GaussNewtonOptimizer::optimize() {
  REASSERT(not models_.empty(), "No error models declared!");

  using SpMat = Eigen::SparseMatrix<double>;
  double residual = std::numeric_limits<double>::max();
  double levenberg_lambda =
      options_.levenberg_a;  // This will be divided out to 1.0 on the first
                             // iteration.
  SpMat error_jacobian{errors_dof_, parameters_dof_};
  Eigen::VectorXd delta{Eigen::VectorXd::Zero(parameters_dof_)};

  for (int ii = 0; ii < options_.max_iterations; ++ii) {
    Eigen::VectorXd errors{Eigen::VectorXd::Zero(errors_dof_)};
    std::vector<Eigen::Triplet<double>> error_jacobian_triplets;

    for (const auto &[model_key, model] : models_) {
      REASSERT(model != nullptr, "Invalid model!");
      const int error_offset = errors_offset(model_key);
      const int error_dof = errors_dof(model_key);

      (*model)(
          errors.segment(error_offset, error_dof),
          [this, error_offset, error_dof, &error_jacobian_triplets](
              const ParameterKey &param_key,
              const int row,
              const int col,
              const double value) {
            const int parameter_offset = parameters_offset(param_key);
            const int parameter_dof = parameters_dof(param_key);

            REASSERT(row >= 0 and row < error_dof, "Row index out of bounds!");
            REASSERT(
                col >= 0 and col < parameter_dof,
                "Col index out of bounds!");

            error_jacobian_triplets.emplace_back(
                row + error_offset,
                col + parameter_offset,
                value);
          });
    }

    double new_residual = errors.squaredNorm();
    if (new_residual > residual) {
      // Then revert to the previous step, which is sitting unmodified in
      // parameters_.next();
      parameters_.swap();
      // The previous step attempt did not reduce the residual, so increase
      // lambda.
      levenberg_lambda = levenberg_lambda * options_.levenberg_b;
      continue;
    }

    residual = new_residual;

    // The previous step did reduce the residual, so reduce lambda.
    levenberg_lambda = levenberg_lambda / options_.levenberg_a;

    error_jacobian.setFromTriplets(
        error_jacobian_triplets.cbegin(),
        error_jacobian_triplets.cend());

    const SpMat JTJ = error_jacobian.transpose() * error_jacobian;
    Eigen::SimplicialCholesky<SpMat> chol(
        JTJ + SpMat(levenberg_lambda * JTJ.diagonal().asDiagonal()));
    delta = chol.solve(error_jacobian.transpose() * errors);

    REASSERT(
        chol.info() != Eigen::NumericalIssue,
        "Non positive definite value for error_jacobian.transpose() * "
        "error_jacobian!");

    for (const auto &step : step_tasks_) {
      step(delta);
    }
    parameters_.swap();

    // We've converged when delta's norm times lambda gets smaller than our
    // tolerance. We max the lambda value with 1 so that small values of lambda
    // don't allow us to terminate while we are still taking large steps.
    if (delta.norm() * std::max(1., levenberg_lambda) < options_.tolerance) {
      return Result{
          .num_iterations = ii + 1,
          .converged = true,
      };
    }
  }
  return Result{
      .num_iterations = options_.max_iterations,
      .converged = false,
  };
}

}  // namespace resim::math
