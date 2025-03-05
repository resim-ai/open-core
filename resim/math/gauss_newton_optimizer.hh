// Copyright 2025 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <Eigen/Dense>
#include <functional>
#include <memory>
#include <vector>

#include "resim/assert/assert.hh"
#include "resim/utils/double_buffer.hh"

namespace resim::math {

// Forward declarations for types used by the GaussNewtonOptimizer. These
// classes are described in detail below.
struct AbstractParameterBlock;

template <typename T>
struct ParameterBlock;

class ErrorModel;

//
// This class contains a generic implementation of the Gauss-Newton method with
// Levenberg-Marquardt step modifications to prohibit divergence. The method is
// fully described in this document: https://ethaneade.com/optimization.pdf
// In particular, it is trying to find a set of parameters theta that minimizes
// the squared norm of the vector returned by an error model e(theta).
//
// Here's how you use it:
//
// First, a user should define the parameters that they want to optimize. These
// are defined in parameter blocks. Each block contains a related set of
// parameters with a common type. For example a block may be a time series of
// spline control points representing the ego's trajectory. Alternatively, it
// could be a set of estimated positions for static geometry. The abstraction
// of blocks is pretty flexible, with the only requirements being that each
// block be identified by a common string key and have a common type for all
// elements. Blocks are represented by the ParameterBlock<T> template and are
// owned by the GaussNewtonOptimizer itself by an abstract pointer to
// AbstractParameterBlock. When setting up the optimizer, the user should
// register them using register_parameters(). They can be obtained using the
// get_parameters() member function template which requires the user to know
// the expected type. The only requirements on the type T are that it define
// the following:
//
//  - DOF: A static constexpr int member containing the number of optimizable
//         degrees of freedom in each instance of the object T.
//
//  - accumulate(): A member function which can take an
//                  Eigen::VectorBlock<Eigen::VectorXd> with DOF elements and
//                  "add" it to the current value of T to return another T
//                  object. If T represents a vector, this is just simple
//                  addition. However, use of the Lie group exponential is
//                  typical more generally if  T is in a Lie group.
//
// These minimal requirements are needed so that we can correctly allocate the
// correct space in the error vector and Jacobian for each block and so that we
// can apply updates to the parameters as we iterate.
//
// Once a user has defined the parameter blocks they need, they should register
// error models (objects inheriting from ErrorModel) that each compute a block
// of the error vector along with its Jacobians with respect to the relevant
// parameter blocks. These error models are typically constructed with a
// reference to the optimizer along with the keys for the parameter blocks they
// need to access. This way, when they are queried using operator(), they can
// access the parameters they need using get_parameters() and then populate the
// correct blocks of the Jacobian using the JacobianWriter object they are
// passed. *For reasons described in the document cited above, the Jacobian is
// the negative of de(theta)/dtheta if the error is given by e(theta)*. The
// optimizer is architected such that each ErrorModel does not have to think
// about the global offsets of the parameter blocks or the parts of the error
// block that it is dealing with. This is managed wholly by the
// GaussNewtonOptimizer object. Each error model also has a unique key. A user
// should register new error models with register_error_model().
//
class GaussNewtonOptimizer {
 public:
  struct Options {
    static constexpr int DEFAULT_MAX_ITERS = 100;
    static constexpr double DEFAULT_TOLERANCE = 1e-7;
    // Convergence criteria
    int max_iterations = DEFAULT_MAX_ITERS;
    double tolerance = DEFAULT_TOLERANCE;

    // Levenberg Parameters
    static constexpr double DEFAULT_A = 2.;
    static constexpr double DEFAULT_B = 10.;
    double levenberg_a = DEFAULT_A;
    double levenberg_b = DEFAULT_B;
  };

  // A struct for the results.
  struct Result {
    int num_iterations = -1;
    bool converged = false;
  };

  using ParameterKey = std::string;
  using ModelKey = std::string;

  // Constructor
  explicit GaussNewtonOptimizer(const Options &options) : options_{options} {}

  // Register a parameter block with the given key.
  // @param[in] key - A distinct key for this parameter block.
  // @param[in] initial_values - Initial values for this parameter block.
  // @throws AssertException if this key is in use.
  template <typename T>
  void register_parameters(
      const std::string &key,
      std::vector<T> initial_values);

  // Get the parameter block with the given key and type T.
  // @param[in] key - A distinct key for this parameter block.
  // @throws AssertException if this key is not found or key's block is not of
  //         type T.
  template <typename T>
  const ParameterBlock<T> &get_parameters(const std::string &key) const;

  // Register an error model with the given key.
  // @param[in] key - A distinct key for this error model.
  // @param[in] error_model - The error model to register.
  // @throws AssertException if this key is in use.
  void register_error_model(
      const std::string &key,
      std::unique_ptr<ErrorModel> error_model);

  // Optimize the internal parameters to minimize the norm of the error vector.
  Result optimize();

 private:
  // Helpers to get the start offset and degrees of freedom for each parameter
  // and error block:
  int parameters_offset(const std::string &key) const;
  int parameters_dof(const std::string &key) const;
  int errors_offset(const std::string &key) const;
  int errors_dof(const std::string &key) const;

  Options options_;

  using ParameterBlockMap =
      std::unordered_map<std::string, std::unique_ptr<AbstractParameterBlock>>;
  using ErrorModelMap =
      std::unordered_map<std::string, std::unique_ptr<ErrorModel>>;

  DoubleBuffer<ParameterBlockMap> parameters_;
  ErrorModelMap models_;

  std::unordered_map<std::string, int> parameter_block_offsets_;
  std::unordered_map<std::string, int> error_block_offsets_;

  int parameters_dof_ = 0;
  int errors_dof_ = 0;

  std::vector<std::function<void(const Eigen::VectorXd &)>> step_tasks_;
};

// Parameter block abstract base class
struct AbstractParameterBlock {
  AbstractParameterBlock() = default;
  virtual ~AbstractParameterBlock() = default;
  AbstractParameterBlock(const AbstractParameterBlock &) = default;
  AbstractParameterBlock(AbstractParameterBlock &&) = default;
  AbstractParameterBlock &operator=(const AbstractParameterBlock &) = default;
  AbstractParameterBlock &operator=(AbstractParameterBlock &&) = default;

  virtual int dof() const = 0;
};

// Parameter block class template.
template <typename T>
struct ParameterBlock : public AbstractParameterBlock {
  int dof() const override { return data.size() * T::DOF; }
  std::vector<T> data;
};

// An error model for use with the GaussNewtonOptimizer as described above.
class ErrorModel {
 public:
  using ParameterKey = GaussNewtonOptimizer::ParameterKey;
  using JacobianWriter = std::function<void(
      const ParameterKey &,  // parameter key
      int,                   // Local row
      int,                   // Local column
      double                 // Value
      )>;

  ErrorModel() = default;
  ErrorModel(const ErrorModel &) = delete;
  ErrorModel(ErrorModel &&) = delete;
  ErrorModel &operator=(const ErrorModel &) = delete;
  ErrorModel &operator=(ErrorModel &&) = delete;
  virtual ~ErrorModel() = default;

  virtual int dof() const = 0;

  // Execute this error model, putting the error into error_block and using the
  // error_jacobian_writer to write the Jacobian.
  // @param[out] error_block - The block to write the error components for this
  //                           model into.
  // @param[in] error_jacobian_writer - The JacobianWriter for the model to use
  //                                    to populate the Jacobian when executing
  //                                    the model.
  virtual void operator()(
      Eigen::VectorBlock<Eigen::VectorXd> error_block,
      const JacobianWriter &error_jacobian_writer) const = 0;
};

template <typename T>
void GaussNewtonOptimizer::register_parameters(
    const std::string &key,
    std::vector<T> initial_values) {
  // Fill both sides of the double buffer
  ParameterBlock<T> block;
  block.data = initial_values;
  for (int ii = 0; ii < 2; ++ii) {
    auto parameter_block = std::make_unique<ParameterBlock<T>>(block);
    REASSERT(
        parameters_.mutable_next()
            .emplace(key, std::move(parameter_block))
            .second,
        "Duplicate parameter key!");
    parameters_.swap();
  }
  REASSERT(
      parameter_block_offsets_.emplace(key, parameters_dof_).second,
      "Duplicate parameter key!");
  parameters_dof_ += parameters_dof(key);

  step_tasks_.emplace_back([this, key](const Eigen::VectorXd &step) {
    const auto &current =
        static_cast<const ParameterBlock<T> &>(*parameters_.current().at(key));

    const int offset = parameters_offset(key);

    auto &next =
        static_cast<ParameterBlock<T> &>(*parameters_.mutable_next().at(key));

    for (int ii = 0; ii < current.data.size(); ++ii) {
      next.data.at(ii) = current.data.at(ii).accumulate(
          step.template segment<T::DOF>(offset + T::DOF * ii));
    }
  });
}

template <typename T>
const ParameterBlock<T> &GaussNewtonOptimizer::get_parameters(
    const std::string &key) const {
  REASSERT(parameters_.current().contains(key), "Key not found!");
  const ParameterBlock<T> *maybe_parameter_block{
      dynamic_cast<const ParameterBlock<T> *>(
          parameters_.current().at(key).get())};
  REASSERT(maybe_parameter_block != nullptr);
  return *maybe_parameter_block;
}

}  // namespace resim::math
