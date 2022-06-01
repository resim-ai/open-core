#include "transforms/liegroup_test_helpers.hh"

#include <Eigen/Dense>
#include <algorithm>
#include <vector>

#include "transforms/so3.hh"

namespace resim {
namespace transforms {

template <typename Vector>
std::vector<Vector> make_test_vectors() {
  constexpr int TEST_ELEMENT_COUNT = 7;
  std::vector<Vector> elements;
  // Add a zero element.
  elements.push_back(Vector::Zero());
  // Add a ones element.
  elements.push_back(Vector::Ones());
  // Add a negative ones element.
  elements.push_back(-Vector::Ones());
  // Populate the remainder with random elements.
  constexpr unsigned int SEED = 42;
  srand(SEED);
  for (int i = elements.size(); i < TEST_ELEMENT_COUNT; ++i) {
    elements.push_back(Vector::Random());
  }
  srand(1);
  elements.resize(TEST_ELEMENT_COUNT);
  return elements;
}

template <typename Liegroup>
std::vector<typename Liegroup::TangentVector> make_test_algebra_elements() {
  return make_test_vectors<typename Liegroup::TangentVector>();
}

template <typename Liegroup>
std::vector<Liegroup> make_test_group_elements() {
  const std::vector<typename Liegroup::TangentVector> algebra_elements =
      make_test_algebra_elements<Liegroup>();
  std::vector<Liegroup> group_elements;
  group_elements.resize(algebra_elements.size());
  std::transform(
      algebra_elements.begin(),
      algebra_elements.end(),
      group_elements.begin(),
      [](const typename Liegroup::TangentVector &alg) -> Liegroup {
        return Liegroup::exp(alg);
      });
  return group_elements;
}

template std::vector<Eigen::Vector3d> make_test_vectors<Eigen::Vector3d>();
template std::vector<SO3::TangentVector> make_test_algebra_elements<SO3>();
template std::vector<SO3> make_test_group_elements<SO3>();

}  // namespace transforms
}  // namespace resim
