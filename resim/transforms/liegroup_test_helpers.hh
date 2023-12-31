// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <vector>

namespace resim::transforms {

namespace detail {
constexpr unsigned MIN_TEST_ELEMENTS = 7;
}

// Builds and returns a vector containing a fixed number of Eigen nx1 matrices
// (Vectors) that are useful for testing. The vectors are a mixture of edge
// cases (e.g. all zeros, all negative one) and randomly populated elements.
// @param[in] count - Optionally, the number of vectors to return. The default
//                    and the minimum are both seven. If you request less the
//                    function will check-fail.
template <typename Vector>
std::vector<Vector> make_test_vectors(
    unsigned count = detail::MIN_TEST_ELEMENTS);

// Builds and returns a vector containing a fixed number of LieGroup tangent
// vectors (algebra elements) that are useful for testing. The tangent vectors
// are a mixture of edge cases (e.g. all zeros, all negative one) and randomly
// populated elements.
// @param[in] count - Optionally, the number of elements to return. The default
//                    and the minimum are both seven. If you request less the
//                    function will check-fail.
template <typename Group>
std::vector<typename Group::TangentVector> make_test_algebra_elements(
    unsigned count = detail::MIN_TEST_ELEMENTS);

// Builds and returns a vector containing a fixed number of LieGroup objects
// that are useful for testing. The LieGroup objects are built by exponentiation
// on the algebra elements returned by make_test_algebra_elements().
// @param[in] count - Optionally, the number of groups to return. The default
//                    and the minimum are both seven. If you request less the
//                    function will check-fail.
template <typename Group>
std::vector<Group> make_test_group_elements(
    unsigned count = detail::MIN_TEST_ELEMENTS);
}  // namespace resim::transforms
