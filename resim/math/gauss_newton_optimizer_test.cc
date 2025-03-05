// Copyright 2025 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/math/gauss_newton_optimizer.hh"

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <random>

#include "resim/assert/assert.hh"
#include "resim/math/polynomial.hh"

namespace resim::math {

namespace {

// A parameter type containing a double with a single degree of freedom.
struct OptimizerDouble {
  static constexpr int DOF = 1;
  explicit OptimizerDouble(const double value) : value{value} {}
  OptimizerDouble(const OptimizerDouble &) = default;
  OptimizerDouble(OptimizerDouble &&) = default;
  OptimizerDouble &operator=(const OptimizerDouble &) = default;
  OptimizerDouble &operator=(OptimizerDouble &&) = default;
  ~OptimizerDouble() = default;

  OptimizerDouble accumulate(
      const Eigen::Ref<const Eigen::VectorXd> &delta) const {
    REASSERT(delta.size() == 1U);

    return OptimizerDouble{value + delta.x()};
  }

  double value = 0.;
};

// An error model that penalizes the deviation from the given set of points
// from a quadratic curve defined by the paremeters referenced by the given
// keys.
class QuadraticErrorModel : public ErrorModel {
 public:
  QuadraticErrorModel(
      const GaussNewtonOptimizer &optimizer,
      std::string quadratic_key,
      std::string linear_key,
      std::string constant_key,
      std::vector<Eigen::Vector2d> points)
      : optimizer_{optimizer},
        quadratic_key_{std::move(quadratic_key)},
        linear_key_{std::move(linear_key)},
        constant_key_{std::move(constant_key)},
        points_{std::move(points)} {}

  int dof() const override { return static_cast<int>(points_.size()); }

  void operator()(
      Eigen::VectorBlock<Eigen::VectorXd> error_block,
      const JacobianWriter &error_jacobian_writer) const override {
    const auto constant_block =
        optimizer_.get_parameters<OptimizerDouble>(constant_key_);
    const auto linear_block =
        optimizer_.get_parameters<OptimizerDouble>(linear_key_);
    const auto quadratic_block =
        optimizer_.get_parameters<OptimizerDouble>(quadratic_key_);

    EXPECT_EQ(constant_block.data.size(), 1U);
    EXPECT_EQ(linear_block.data.size(), 1U);
    EXPECT_EQ(quadratic_block.data.size(), 1U);

    const double a = constant_block.data.front().value;
    const double b = linear_block.data.front().value;
    const double c = quadratic_block.data.front().value;

    for (int ii = 0; ii < points_.size(); ++ii) {
      const Eigen::Vector2d &point = points_.at(ii);
      error_block(ii) = point.y() - (a + (b + c * point.x()) * point.x());

      error_jacobian_writer(constant_key_, ii, 0, 1.);
      error_jacobian_writer(linear_key_, ii, 0, point.x());
      error_jacobian_writer(quadratic_key_, ii, 0, point.x() * point.x());
    }
  }

 private:
  const GaussNewtonOptimizer &optimizer_;
  std::string quadratic_key_;
  std::string linear_key_;
  std::string constant_key_;
  std::vector<Eigen::Vector2d> points_;
};

// An error model that penalizes the deviation from the given set of points
// from a Gaussian PDF curve defined by the paremeters referenced by the given
// keys.
class GaussianErrorModel : public ErrorModel {
 public:
  GaussianErrorModel(
      const GaussNewtonOptimizer &optimizer,
      std::string mean_key,
      std::string stddev_key,
      std::vector<Eigen::Vector2d> points)
      : optimizer_{optimizer},
        mean_key_{std::move(mean_key)},
        stddev_key_{std::move(stddev_key)},
        points_{std::move(points)} {}

  int dof() const override { return static_cast<int>(points_.size()); }

  void operator()(
      Eigen::VectorBlock<Eigen::VectorXd> error_block,
      const JacobianWriter &error_jacobian_writer) const override {
    const auto stddev_block =
        optimizer_.get_parameters<OptimizerDouble>(stddev_key_);
    const auto mean_block =
        optimizer_.get_parameters<OptimizerDouble>(mean_key_);

    EXPECT_EQ(stddev_block.data.size(), 1U);
    EXPECT_EQ(mean_block.data.size(), 1U);

    const double sigma = stddev_block.data.front().value;
    const double mu = mean_block.data.front().value;

    for (int ii = 0; ii < points_.size(); ++ii) {
      const Eigen::Vector2d &point = points_.at(ii);

      const double f_of_x =
          (1. / std::sqrt(2. * M_PI) / sigma) *
          std::exp(-0.5 * std::pow((point.x() - mu) / sigma, 2.));

      error_block(ii) = point.y() - f_of_x;

      // NOLINTBEGIN(readability-magic-numbers)
      error_jacobian_writer(
          stddev_key_,
          ii,
          0,
          -f_of_x / sigma +
              f_of_x * std::pow(point.x() - mu, 2.) / std::pow(sigma, 3.));
      error_jacobian_writer(
          mean_key_,
          ii,
          0,
          f_of_x * (point.x() - mu) / std::pow(mu, 2.));
      // NOLINTEND(readability-magic-numbers)
    }
  }

 private:
  const GaussNewtonOptimizer &optimizer_;
  std::string mean_key_;
  std::string stddev_key_;
  std::vector<Eigen::Vector2d> points_;
};

// An error model that penalizes the deviation of the current parameter from
// zero by the error function e(t) = t / (1 + t). This error function is used
// because ||e(t)||^2 becomes non-convex far from the optimum so we can test
// that our optimizer works in such cases.
class NonConvexErrorModel : public ErrorModel {
 public:
  NonConvexErrorModel(
      const GaussNewtonOptimizer &optimizer,
      std::string parameter_key)
      : optimizer_{optimizer},
        parameter_key_{std::move(parameter_key)} {}

  int dof() const override { return 1; }

  void operator()(
      Eigen::VectorBlock<Eigen::VectorXd> error_block,
      const JacobianWriter &error_jacobian_writer) const override {
    const auto parameter_block =
        optimizer_.get_parameters<OptimizerDouble>(parameter_key_);

    EXPECT_EQ(parameter_block.data.size(), 1U);

    const double val = parameter_block.data.front().value;
    const double abs_val = std::fabs(val);
    error_block(0) = abs_val / (1 + abs_val);

    // NOLINTBEGIN(readability-magic-numbers)
    if (val > 0.) {
      error_jacobian_writer(parameter_key_, 0, 0, -1. / std::pow(1. + val, 2.));
    } else if (val < 0.) {
      error_jacobian_writer(parameter_key_, 0, 0, 1. / std::pow(1. - val, 2.));
    } else {
      error_jacobian_writer(parameter_key_, 0, 0, 1.);
    }
    // NOLINTEND(readability-magic-numbers)
  }

 private:
  const GaussNewtonOptimizer &optimizer_;
  std::string parameter_key_;
};

// A badly behaved error model that we use to check that we catch models that
// try to write to indices that are out of bounds for them.
class BadlyBehavedErrorModel : public ErrorModel {
 public:
  enum class BadSetting {
    NEGATIVE_ROW,
    ROW_TOO_BIG,
    NEGATIVE_COL,
    COL_TOO_BIG,
  };

  BadlyBehavedErrorModel(
      const GaussNewtonOptimizer &optimizer,
      std::string parameter_key,
      const BadSetting bad_setting)
      : optimizer_{optimizer},
        parameter_key_{std::move(parameter_key)},
        setting_{bad_setting} {}

  int dof() const override { return 1; }

  void operator()(
      Eigen::VectorBlock<Eigen::VectorXd> error_block,
      const JacobianWriter &error_jacobian_writer) const override {
    const auto parameter_block =
        optimizer_.get_parameters<OptimizerDouble>(parameter_key_);

    EXPECT_EQ(parameter_block.data.size(), 1U);

    const double val = parameter_block.data.front().value;
    error_block(0) = val;

    switch (setting_) {
      case BadSetting::NEGATIVE_ROW: {
        error_jacobian_writer(parameter_key_, -1, 0, -1.);
        break;
      }
      case BadSetting::ROW_TOO_BIG: {
        error_jacobian_writer(parameter_key_, 2, 0, -1.);
        break;
      }
      case BadSetting::NEGATIVE_COL: {
        error_jacobian_writer(parameter_key_, 0, -1, -1.);
        break;
      }
      case BadSetting::COL_TOO_BIG: {
        error_jacobian_writer(parameter_key_, 0, 2, -1.);
        break;
      }
    }
  }

 private:
  const GaussNewtonOptimizer &optimizer_;
  std::string parameter_key_;
  BadSetting setting_;
};

}  // namespace

TEST(GaussNewtonOptimizerTest, TestPolynomialFit) {
  // SETUP
  constexpr unsigned SEED = 293U;
  std::mt19937 rng{SEED};
  constexpr double LOWER_BOUND = -1.;
  constexpr double UPPER_BOUND = 1.;
  std::uniform_real_distribution<double> dist{-LOWER_BOUND, UPPER_BOUND};

  const double constant = dist(rng);
  const double linear = dist(rng);
  const double quadratic = dist(rng);

  constexpr int QUADRATIC = 2;
  Polynomial<QUADRATIC> polynomial{{constant, linear, quadratic}};

  const GaussNewtonOptimizer::Options options;
  GaussNewtonOptimizer optimizer{options};

  //  Make some sample observations
  constexpr int NUM_OBSERVATIONS = 10;
  std::vector<Eigen::Vector2d> observations;
  for (int ii = 0; ii < NUM_OBSERVATIONS; ++ii) {
    const double x = static_cast<double>(ii) / (NUM_OBSERVATIONS - 1);
    observations.emplace_back(x, polynomial(x));
  }

  for (int ii = 0; ii < NUM_OBSERVATIONS; ++ii) {
    optimizer.register_error_model(
        "observation_" + std::to_string(ii),
        std::make_unique<QuadraticErrorModel>(
            optimizer,
            "quadratic",
            "linear",
            "constant",
            std::vector<Eigen::Vector2d>{observations.at(ii)}));
  }

  std::vector<OptimizerDouble> zero;
  zero.emplace_back(0.);
  optimizer.register_parameters<OptimizerDouble>("quadratic", zero);
  optimizer.register_parameters<OptimizerDouble>("linear", zero);
  optimizer.register_parameters<OptimizerDouble>("constant", zero);

  // ACTION
  const auto result = optimizer.optimize();

  // VERIFICATION
  EXPECT_TRUE(result.converged);
  EXPECT_GT(result.num_iterations, 1);
  EXPECT_LE(result.num_iterations, options.max_iterations);

  const auto constant_block =
      optimizer.get_parameters<OptimizerDouble>("constant");
  const auto linear_block = optimizer.get_parameters<OptimizerDouble>("linear");
  const auto quadratic_block =
      optimizer.get_parameters<OptimizerDouble>("quadratic");

  EXPECT_EQ(constant_block.data.size(), 1U);
  EXPECT_EQ(linear_block.data.size(), 1U);
  EXPECT_EQ(quadratic_block.data.size(), 1U);

  const double a = constant_block.data.front().value;
  const double b = linear_block.data.front().value;
  const double c = quadratic_block.data.front().value;

  constexpr double TOLERANCE = 1e-7;
  EXPECT_NEAR(a, constant, TOLERANCE);
  EXPECT_NEAR(b, linear, TOLERANCE);
  EXPECT_NEAR(c, quadratic, TOLERANCE);
}

TEST(GaussNewtonOptimizerTest, TestGaussianPDFFit) {
  // SETUP
  constexpr unsigned SEED = 293U;
  std::mt19937 rng{SEED};
  constexpr double LOWER_BOUND = -1.;
  constexpr double UPPER_BOUND = 1.;
  std::uniform_real_distribution<double> dist{-LOWER_BOUND, UPPER_BOUND};

  const double stddev = 5.0 * std::pow(dist(rng), 2.0);
  const double mean = 5.0 + dist(rng);

  // NOLINTBEGIN(readability-magic-numbers)
  const auto model = [stddev, mean](const double x) {
    return 1. / sqrt(2. * M_PI) / stddev *
           std::exp(-0.5 * std::pow((x - mean) / stddev, 2.));
  };
  // NOLINTEND(readability-magic-numbers)

  const GaussNewtonOptimizer::Options options;
  GaussNewtonOptimizer optimizer{options};

  //  Make some sample observations
  constexpr int NUM_OBSERVATIONS = 10;
  std::vector<Eigen::Vector2d> observations;
  for (int ii = 0; ii < NUM_OBSERVATIONS; ++ii) {
    const double x = static_cast<double>(ii) / (NUM_OBSERVATIONS - 1);
    observations.emplace_back(x, model(x));
  }

  optimizer.register_error_model(
      "observations",
      std::make_unique<GaussianErrorModel>(
          optimizer,
          "mean",
          "stddev",
          observations));

  std::vector<OptimizerDouble> one;
  one.emplace_back(1.);
  optimizer.register_parameters<OptimizerDouble>("mean", one);
  optimizer.register_parameters<OptimizerDouble>("stddev", one);

  // ACTION
  auto result = optimizer.optimize();

  // VERIFICATION
  EXPECT_TRUE(result.converged);
  EXPECT_GT(result.num_iterations, 1);
  EXPECT_LE(result.num_iterations, options.max_iterations);

  const auto stddev_block = optimizer.get_parameters<OptimizerDouble>("stddev");
  const auto mean_block = optimizer.get_parameters<OptimizerDouble>("mean");

  EXPECT_EQ(stddev_block.data.size(), 1U);
  EXPECT_EQ(mean_block.data.size(), 1U);

  const double sigma = stddev_block.data.front().value;
  const double mu = mean_block.data.front().value;

  constexpr double TOLERANCE = 1e-7;
  EXPECT_NEAR(sigma, stddev, TOLERANCE);
  EXPECT_NEAR(mu, mean, TOLERANCE);
}

TEST(GaussNewtonOptimizerTest, TestGaussianPDFFitNoConverge) {
  // SETUP
  constexpr unsigned SEED = 293U;
  std::mt19937 rng{SEED};
  constexpr double LOWER_BOUND = -1.;
  constexpr double UPPER_BOUND = 1.;
  std::uniform_real_distribution<double> dist{-LOWER_BOUND, UPPER_BOUND};

  const double stddev = 5.0 * std::pow(dist(rng), 2.0);
  const double mean = 5.0 + dist(rng);

  // NOLINTBEGIN(readability-magic-numbers)
  const auto model = [stddev, mean](const double x) {
    return 1. / sqrt(2. * M_PI) / stddev *
           std::exp(-0.5 * std::pow((x - mean) / stddev, 2.));
  };
  // NOLINTEND(readability-magic-numbers)

  const GaussNewtonOptimizer::Options options{
      .max_iterations = 3,
      .tolerance = 1e-7,
  };
  GaussNewtonOptimizer optimizer{options};

  //  Make some sample observations
  constexpr int NUM_OBSERVATIONS = 10;
  std::vector<Eigen::Vector2d> observations;
  for (int ii = 0; ii < NUM_OBSERVATIONS; ++ii) {
    const double x = static_cast<double>(ii) / (NUM_OBSERVATIONS - 1);
    observations.emplace_back(x, model(x));
  }

  optimizer.register_error_model(
      "observations",
      std::make_unique<GaussianErrorModel>(
          optimizer,
          "mean",
          "stddev",
          observations));

  std::vector<OptimizerDouble> initial_mean;
  initial_mean.emplace_back(1.);
  std::vector<OptimizerDouble> initial_stddev;
  initial_stddev.emplace_back(1.);
  optimizer.register_parameters<OptimizerDouble>("mean", initial_mean);
  optimizer.register_parameters<OptimizerDouble>("stddev", initial_stddev);

  // ACTION
  auto result = optimizer.optimize();

  // VERIFICATION
  EXPECT_FALSE(result.converged);
  EXPECT_EQ(result.num_iterations, options.max_iterations);
}

TEST(GaussNewtonOptimizerTest, TestNonConvexErrorModel) {
  // SETUP
  const GaussNewtonOptimizer::Options options;
  GaussNewtonOptimizer optimizer{options};

  optimizer.register_error_model(
      "model",
      std::make_unique<NonConvexErrorModel>(optimizer, "value"));

  std::vector<OptimizerDouble> initial_value;
  constexpr double VALUE_IN_CONCAVE_DOWN_AREA = 5.;
  initial_value.emplace_back(VALUE_IN_CONCAVE_DOWN_AREA);
  optimizer.register_parameters<OptimizerDouble>("value", initial_value);

  // ACTION
  auto result = optimizer.optimize();

  // VERIFICATION
  EXPECT_TRUE(result.converged);
  EXPECT_GT(result.num_iterations, 1);
  EXPECT_LE(result.num_iterations, options.max_iterations);

  const auto param_block = optimizer.get_parameters<OptimizerDouble>("value");
  EXPECT_EQ(param_block.data.size(), 1U);
  const double param = param_block.data.front().value;

  constexpr double TOLERANCE = 1e-7;
  EXPECT_NEAR(param, 0, TOLERANCE);
}

TEST(GaussNewtonOptimizerTest, TestBadlyBehavedModelThrows) {
  // SETUP
  using Setting = BadlyBehavedErrorModel::BadSetting;
  const GaussNewtonOptimizer::Options options;

  for (const auto bad_setting :
       {Setting::NEGATIVE_ROW,
        Setting::ROW_TOO_BIG,
        Setting::NEGATIVE_COL,
        Setting::COL_TOO_BIG}) {
    GaussNewtonOptimizer optimizer{options};

    optimizer.register_error_model(
        "model",
        std::make_unique<BadlyBehavedErrorModel>(
            optimizer,
            "value",
            bad_setting));

    std::vector<OptimizerDouble> initial_value;
    initial_value.emplace_back(1.);
    optimizer.register_parameters<OptimizerDouble>("value", initial_value);

    // ACTION / VERIFICATION
    EXPECT_THROW(optimizer.optimize(), AssertException);
  }
}

TEST(GaussNewtonOptimizerTest, TestThrowOnIncomplete) {
  // Nothing set

  // SETUP
  const GaussNewtonOptimizer::Options options;
  GaussNewtonOptimizer optimizer{options};

  // ACTION / VERIFICATION
  EXPECT_THROW(optimizer.optimize(), AssertException);

  // Model missing

  // SETUP
  std::vector<OptimizerDouble> initial_value;
  initial_value.emplace_back(1.);
  optimizer.register_parameters<OptimizerDouble>("value", initial_value);

  // ACTION / VERIFICATION
  EXPECT_THROW(optimizer.optimize(), AssertException);

  // Parameter missing
  {
    // SETUP
    const GaussNewtonOptimizer::Options options;
    GaussNewtonOptimizer optimizer{options};

    optimizer.register_error_model(
        "model",
        std::make_unique<NonConvexErrorModel>(optimizer, "value"));

    // ACTION / VERIFICATION
    EXPECT_THROW(optimizer.optimize(), AssertException);
  }
}

TEST(GaussNewtonOptimizerTest, TestNonPDFailure) {
  // SETUP
  const GaussNewtonOptimizer::Options options;
  GaussNewtonOptimizer optimizer{options};

  optimizer.register_error_model(
      "model",
      std::make_unique<NonConvexErrorModel>(optimizer, "value"));

  std::vector<OptimizerDouble> initial_value;
  initial_value.emplace_back(1.);
  optimizer.register_parameters<OptimizerDouble>("value", initial_value);
  optimizer.register_parameters<OptimizerDouble>("unused_value", initial_value);

  // ACTION / VERIFICATION
  EXPECT_THROW(optimizer.optimize(), AssertException);
}

TEST(GaussNewtonOptimizerTest, TestKeyCollisions) {
  // SETUP
  const GaussNewtonOptimizer::Options options;
  GaussNewtonOptimizer optimizer{options};

  optimizer.register_error_model(
      "model",
      std::make_unique<NonConvexErrorModel>(optimizer, "value"));
  std::vector<OptimizerDouble> initial_value;
  initial_value.emplace_back(0.);
  optimizer.register_parameters<OptimizerDouble>("value", initial_value);

  // ACTION / VERIFICATION
  EXPECT_THROW(
      optimizer.register_error_model(
          "model",
          std::make_unique<NonConvexErrorModel>(optimizer, "value")),
      AssertException);
  EXPECT_THROW(
      optimizer.register_parameters<OptimizerDouble>("value", initial_value),
      AssertException);
}

TEST(GaussNewtonOptimizerTest, TestNullErrorModel) {
  // SETUP
  const GaussNewtonOptimizer::Options options;
  GaussNewtonOptimizer optimizer{options};

  // ACTION / VERIFICATION
  EXPECT_THROW(
      optimizer.register_error_model("model", nullptr),
      AssertException);
}

}  // namespace resim::math
