#include "transforms/liegroup_test_helpers.hh"
#include "transforms/so3.hh"

#include <algorithm>
#include <vector>

namespace resim {
namespace transforms {

template <typename Liegroup>
std::vector<typename Liegroup::TangentVector> make_test_algebra_elements() {
    constexpr int TEST_ELEMENT_COUNT = 7;
    std::vector<typename Liegroup::TangentVector> elements;
    // Add a zero element.
    elements.push_back(Liegroup::TangentVector::Zero());
    // Add a ones element.
    elements.push_back(Liegroup::TangentVector::Ones());
    // Add a negative ones element.
    elements.push_back(-Liegroup::TangentVector::Ones());
    // Populate the remainder with random elements.
    constexpr unsigned int SEED = 42;
    srand(SEED);
    for (int i = elements.size(); i < TEST_ELEMENT_COUNT; ++i) {
        elements.push_back(Liegroup::TangentVector::Random());
    }
    srand(1);
    elements.resize(TEST_ELEMENT_COUNT);
    return elements;
}

template <typename Liegroup>
std::vector<Liegroup> make_test_group_elements() {
    const std::vector<typename Liegroup::TangentVector> algebra_elements 
        = make_test_algebra_elements<Liegroup>();
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

template std::vector<SO3::TangentVector> make_test_algebra_elements<SO3>();
template std::vector<SO3> make_test_group_elements<SO3>();
} // namespace transforms
} // namespace resim
