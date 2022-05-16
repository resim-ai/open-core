#pragma once

#include <vector>

namespace resim {
namespace transforms {

// Builds and returns a vector containint a fixed number of Eigen nx1 matrices 
// (Vectors) that are useful for testing. The vectors are a mixture of edge 
// cases (e.g. all zeros, all negative one) and randomly populated elements.
template <typename Vector>
std::vector<Vector> make_test_vectors();

// Builds and returns a vector containing a fixed number of Liegroup tangent 
// vectors (algebra elements) that are useful for testing. The tangent vectors 
// are a mixture of edge cases (e.g. all zeros, all negative one) and randomly 
// populated elements.
template <typename Liegroup>
std::vector<typename Liegroup::TangentVector> make_test_algebra_elements();

// Builds and returns a vector containing a fixed number of Liegroup objects
// that are useful for testing. The Liegroup objects are built by exponentiation
// on the algebra elements returned by mate_test_algebra_elements().
template <typename Liegroup>
std::vector<Liegroup> make_test_group_elements();

} // namespace transforms
} // namespace resim
