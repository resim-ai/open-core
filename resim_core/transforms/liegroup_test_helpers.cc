#include "resim_core/transforms/liegroup_test_helpers.hh"

#include <Eigen/Dense>
#include <algorithm>
#include <random>
#include <vector>

#include "resim_core/transforms/se3.hh"
#include "resim_core/transforms/so3.hh"
#include "resim_core/utils/random_matrix.hh"
#include "resim_core/utils/type.hh"

namespace resim::transforms {

namespace {

template <typename Vector, typename Rng>
Vector make_large_vector(BasicType<Vector> vec_type, Rng &&rng) {
  constexpr double LARGE = 1E6;
  return testing::random_vector<Vector>(rng) * LARGE;
}

template <typename Rng>
SO3::TangentVector make_large_vector(
    BasicType<SO3::TangentVector> vec_type,
    Rng &&rng) {
  // Close to Pi
  constexpr double NEARLY_PI = M_PI - 0.01;
  constexpr double SQRT_3 = 1.732;
  constexpr double LRG_ROT = NEARLY_PI / SQRT_3;
  return SO3::TangentVector{LRG_ROT, LRG_ROT, -LRG_ROT};
}

template <typename Rng>
SE3::TangentVector make_large_vector(
    BasicType<SE3::TangentVector> vec_type,
    Rng &&rng) {
  constexpr double LARGE = 1E6;
  return SE3::tangent_vector_from_parts(
      make_large_vector(TypeC<SO3::TangentVector>, rng),
      testing::random_vector<Eigen::Vector3d>(rng) * LARGE);
}

}  // namespace

template <typename Vector>
std::vector<Vector> make_test_vectors() {
  // Make random seed determistic
  constexpr unsigned int SEED = 42;
  std::mt19937 rng{SEED};
  // How many test elements to make.
  constexpr int TEST_ELEMENT_COUNT = 7;
  std::vector<Vector> elements;
  // Add a zero element.
  elements.push_back(Vector::Zero());
  // Add a ones element.
  elements.push_back(Vector::Ones());
  // Add a negative ones element.
  elements.push_back(-Vector::Ones());
  // Add a large element.
  elements.push_back(make_large_vector(TypeC<Vector>, rng));
  // Add a tiny numbers element.
  constexpr double TINY = 1E-6;
  elements.push_back(testing::random_vector<Vector>(rng) * TINY);
  // Populate the remainder with random elements.
  for (int i = elements.size(); i < TEST_ELEMENT_COUNT; ++i) {
    elements.push_back(testing::random_vector<Vector>(rng));
  }
  elements.resize(TEST_ELEMENT_COUNT);
  return elements;
}

template <typename Group>
std::vector<typename Group::TangentVector> make_test_algebra_elements() {
  return make_test_vectors<typename Group::TangentVector>();
}

template <typename Group>
std::vector<Group> make_test_group_elements() {
  const std::vector<typename Group::TangentVector> algebra_elements =
      make_test_algebra_elements<Group>();
  std::vector<Group> group_elements;
  group_elements.resize(algebra_elements.size());
  std::transform(
      algebra_elements.begin(),
      algebra_elements.end(),
      group_elements.begin(),
      [](const typename Group::TangentVector &alg) -> Group {
        return Group::exp(alg);
      });
  return group_elements;
}

template std::vector<Eigen::Vector3d> make_test_vectors<Eigen::Vector3d>();
template std::vector<SO3::TangentVector> make_test_algebra_elements<SO3>();
template std::vector<SO3> make_test_group_elements<SO3>();
template std::vector<SE3::TangentVector> make_test_algebra_elements<SE3>();
template std::vector<SE3> make_test_group_elements<SE3>();

}  // namespace resim::transforms
