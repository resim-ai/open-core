#pragma once

#include <vector>

namespace resim::transforms {

// Builds and returns a vector containint a fixed number of Eigen nx1 matrices
// (Vectors) that are useful for testing. The vectors are a mixture of edge
// cases (e.g. all zeros, all negative one) and randomly populated elements.
template <typename Vector>
std::vector<Vector> make_test_vectors();

// Builds and returns a vector containing a fixed number of LieGroup tangent
// vectors (algebra elements) that are useful for testing. The tangent vectors
// are a mixture of edge cases (e.g. all zeros, all negative one) and randomly
// populated elements.
template <typename Group>
std::vector<typename Group::TangentVector> make_test_algebra_elements();

// Builds and returns a vector containing a fixed number of LieGroup objects
// that are useful for testing. The LieGroup objects are built by exponentiation
// on the algebra elements returned by mate_test_algebra_elements().
template <typename Group>
std::vector<Group> make_test_group_elements();

}  // namespace resim::transforms
