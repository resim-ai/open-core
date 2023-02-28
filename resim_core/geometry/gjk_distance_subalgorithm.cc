#include "resim_core/geometry/gjk_distance_subalgorithm.hh"

#include <cstdint>
#include <limits>
#include <optional>

#include "resim_core/assert/assert.hh"
#include "resim_core/utils/inout.hh"
#include "resim_core/utils/integer_power.hh"

namespace resim::geometry::convex_helpers {
namespace {

// This helper visits a subset of size set_size (represented by a bitset where
// bits that are 1 represent elements that are in the set and bits that are zero
// represent elements that are *not* included.
// subset - The subset to iterate through, expressed as a bitset
// set_size - The size of the set that subset is a subset of.
// f - A callable which is called with the element indices which are present in
//     the subset.
template <typename Callable_t>
void for_each_in_set(
    const uint64_t subset,
    const uint64_t set_size,
    Callable_t &&f) {
  constexpr uint64_t ONE = 0x1;
  for (int ii = 0U; ii < set_size; ++ii) {
    if (subset bitand ONE << ii) {
      f(ii);
    }
  }
}

// This helper visits the complement of a subset of size set_size (represented
// by a bitset where bits that are 1 represent elements that are in the set and
// bits that are zero represent elements that are *not* included.
// subset - The subset whose complement we want to iterate through, expressed as
//          a bitset
// set_size - The size of the set that subset is a subset of.
// f - A callable which is called with the element indices which are present in
//     the subsets complement.
template <typename Callable_t>
void for_each_not_in_set(
    const uint64_t subset,
    const uint64_t set_size,
    Callable_t &&f) {
  constexpr uint64_t ONE = 0x1;
  for (int ii = 0U; ii < set_size; ++ii) {
    if (not(subset bitand ONE << ii)) {
      f(ii);
    }
  }
}

// This helper visits the "successors" of a given subset. A successor is a new
// subset (i.e. not equal to the original subset) which can be obtained by
// including one more element of the set in the original subset. This function
// visits all possible successor subsets.
// subset - The subset whose successors we want to iterate through, expressed as
//          a bitset
// set_size - The size of the set that subset is a subset of.
// f - A callable which is called with the element indices we are adding and the
//     bitset representations of the successor subsets.
template <typename Callable_t>
void for_each_successor(
    const uint64_t subset,
    const uint64_t set_size,
    Callable_t &&f) {
  constexpr uint64_t ONE = 0x1;
  for_each_not_in_set(subset, set_size, [&](const uint64_t ii) {
    f(ii, subset bitor ONE << ii);
  });
}

// The type of the matrix we use to store the cofactor values (the Delta values)
// as described in Section V of
// https://drive.google.com/file/d/1A8j4b-Wknf6eAz8zh4YMKNLowfTuxXWD/view.
using DeltaMatrix = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;

// A helper to make a checked narrowing conversion to Eigen::Index from uint64_t
Eigen::Index index_from_uint64(const uint64_t x) {
  constexpr auto ERROR_MESSAGE = "Could not convert uint64_t to Eigen::Index!";
  // Lower bound doesn't need to be checked since it's below zero and
  // unsigned ints always satisfy this.
  REASSERT(x <= std::numeric_limits<Eigen::Index>::max(), ERROR_MESSAGE);
  return static_cast<Eigen::Index>(x);
}

// A helper to query a DeltaMatrix using uint64_t indices rather than
// Eigen::Index
double &
query_deltas(const uint64_t ii, const uint64_t jj, InOut<DeltaMatrix> deltas) {
  return (*deltas)(index_from_uint64(ii), index_from_uint64(jj));
}

// Overload of the above which works for a const delta matrix.
double
query_deltas(const uint64_t ii, const uint64_t jj, const DeltaMatrix &deltas) {
  return deltas(index_from_uint64(ii), index_from_uint64(jj));
}

// Populate the base cases for the recursive relation described for the cofactor
// (Delta) values as described in Section V of
// https://drive.google.com/file/d/1A8j4b-Wknf6eAz8zh4YMKNLowfTuxXWD/view.
// @param[in] num_vertices - The number of vertices in our simplex.
// @param[inout] deltas - The cofactors matrix we're populating.
void populate_base_cases(
    const uint64_t num_vertices,
    InOut<DeltaMatrix> deltas) {
  for (uint64_t ii = 0U; ii < num_vertices; ++ii) {
    constexpr uint64_t ONE = 1U;
    query_deltas((ONE << ii) - 1U, ii, deltas) = 1.;
  }
}

// A simple helper that determines whether all cofactors/deltas corresponding to
// the given subset are strictly positive.
// @param[in] subset - The subset whose cofactors we're inspecting.
// @param[in] num_vertices - The number of vertices in our simplex.
// @param[in] deltas - The cofactors we're inspecting.
bool all_deltas_positive(
    const uint64_t subset,
    const uint64_t num_vertices,
    const DeltaMatrix &deltas) {
  bool result = true;
  for_each_in_set(subset, num_vertices, [&](const uint64_t ii) {
    if (query_deltas(subset - 1U, ii, deltas) <= 0.0) {
      result = false;
    }
  });
  return result;
}

// A simple helper that determines whether all cofactors/deltas corresponding to
// the successors of the subset are non-positive
// @param[in] subset - The subset whose successors' cofactors we're inspecting.
// @param[in] num_vertices - The number of vertices in our simplex.
// @param[in] deltas - The cofactors we're inspecting.
bool all_successors_negative(
    const uint64_t subset,
    const uint64_t num_vertices,
    const DeltaMatrix &deltas) {
  bool result = true;
  for_each_successor(
      subset,
      num_vertices,
      [&](const uint64_t new_vertex, const uint64_t successor) {
        if (query_deltas(successor - 1U, new_vertex, deltas) > 0.0) {
          result = false;
        }
      });
  return result;
}

// Helper to find and return the subset of the given simplex which is closest to
// the origin using the algorithm from Section V of
// https://drive.google.com/file/d/1A8j4b-Wknf6eAz8zh4YMKNLowfTuxXWD/view. We
// populate the cofactors matrix as we do this.
// @param[in] simplex - The simplex to find the closest subset of.
// @param[in] num_subsets - The number of subsets of this simplex. Should
//                          obviously be 2**simplex.size(), but we already
//                          compute it it the calling code, so we just pass it
//                          in.
// @param[in] only_populate_deltas - Only compute deltas and return failure.
// @param[inout] delta_inout - The cofactors matrix we're populating. It is
//                             assumed that the base case values have been
//                             populated by populate_base_cases().
// @returns - The closest subset to the origin if it can be found by this
//            technique. As the paper notes, there are cases where this
//            procedure can fail. In such cases, we return 0U as a sentinel
//            value so the client below can use the backup procedure noted in
//            the paper.
constexpr uint64_t FAILURE_SENTINEL = 0U;
template <int DIM>
uint64_t find_closest_subset(
    const Simplex<DIM> &simplex,
    const uint64_t num_subsets,
    const bool only_populate_deltas,
    InOut<DeltaMatrix> delta_inout) {
  const uint64_t num_vertices = simplex.size();
  auto &deltas = *delta_inout;

  // Iterate through all non-empty subsets of the simplex.
  for (uint64_t subset = 1U; subset < num_subsets; ++subset) {
    // Populate the cofactors for all successors using the recurrence relation.
    for_each_successor(
        subset,
        num_vertices,
        [&](const uint64_t new_vertex, const uint64_t successor) {
          // We arbitrarily pick the first k we find in the simplex like the
          // paper does.
          std::optional<uint64_t> kk;
          for_each_in_set(subset, num_vertices, [&](const uint64_t ii) {
            if (not kk) {
              kk = ii;
            }
            query_deltas(successor - 1U, new_vertex, delta_inout) +=
                query_deltas(subset - 1U, ii, deltas) *
                (simplex.at(ii).dot(simplex.at(*kk)) -
                 simplex.at(ii).dot(simplex.at(new_vertex)));
          });
        });
    // Terminate if we've reached the termination condition
    if (not only_populate_deltas and
        all_deltas_positive(subset, num_vertices, deltas) and
        all_successors_negative(subset, num_vertices, deltas)) {
      return subset;
    }
  }
  return FAILURE_SENTINEL;
}

// A simple helper which computes the closest point to the origin in the given
// subset's affine space. If all of the cofactors/deltas for this subset are
// positive, this will be the closest point on the simplex.
// @param[in] simplex - The simplex the subset is a subset of.
// @param[in] subset - The subset/subsimplex we're trying to find the closest
//                     point on.
// @param[in] deltas - The cofactors we're using to compute the closest
//                     point. Assumed to be populated for all members of the
//                     subset.
template <int DIM>
Eigen::Matrix<double, DIM, 1> compute_closest_point(
    const Simplex<DIM> &simplex,
    const uint64_t subset,
    const DeltaMatrix &deltas) {
  const uint64_t num_vertices = simplex.size();
  Eigen::Matrix<double, DIM, 1> closest_point{
      Eigen::Matrix<double, DIM, 1>::Zero()};

  double deltas_sum = 0.0;
  for_each_in_set(subset, num_vertices, [&](const uint64_t ii) {
    const double current_delta = query_deltas(subset - 1U, ii, deltas);
    deltas_sum += current_delta;
    closest_point += current_delta * simplex.at(ii);
  });
  closest_point /= deltas_sum;
  return closest_point;
}

// Simple helper to construct a subsimplex of the given simplex containing the
// points of the given subset.
// @param[in] simplex - The simplex the subset is a subset of.
// @param[in] subset - The subset/subsimplex we're trying to create a Simplex
//                     object for.
template <int DIM>
Simplex<DIM> compute_new_simplex(
    const Simplex<DIM> &simplex,
    const uint64_t subset) {
  Simplex<DIM> new_simplex;
  for_each_in_set(subset, simplex.size(), [&](const uint64_t ii) {
    new_simplex.push_back(simplex.at(ii));
  });
  return new_simplex;
}

// This function is the backup procedure described in section VI of the attached
// paper. It should be used when the main procedure fails, which can happen when
// the simplex is not affine independent. It isn't used normally since it is
// slower.
// @param[in] simplex - The simplex we're finding the closest point on.
// @param[in] num_subsets - The number of subsets of the simplex.
// @param[in] deltas - The populated cofactors matrix. This should be fully
//                     populated if the main procedure ran to completion.
// @returns The full DistanceResult for the algorithm.
template <int DIM>
DistanceResult<DIM> backup_distance_procedure(
    const Simplex<DIM> simplex,
    const uint64_t num_subsets,
    const DeltaMatrix &deltas) {
  // Go through all subsets with all-positive cofactors and pick the closest
  // point from among these.
  uint64_t result_subset = 0U;
  double closest_distance = std::numeric_limits<double>::max();
  DistanceResult<DIM> result;
  for (uint64_t subset = 0; subset < num_subsets; ++subset) {
    if (all_deltas_positive(subset, simplex.size(), deltas)) {
      const Eigen::Matrix<double, DIM, 1> current_closest_point{
          compute_closest_point(simplex, subset, deltas)};
      const double current_closest_distance = current_closest_point.norm();
      if (current_closest_distance < closest_distance) {
        closest_distance = current_closest_distance;
        result.closest_point = current_closest_point;
        result_subset = subset;
      }
    }
  }
  result.simplex = compute_new_simplex(simplex, result_subset);
  return result;
}

}  // namespace

template <int DIM>
DistanceResult<DIM> distance_subalgorithm(
    const Simplex<DIM> &simplex,
    const testing::Algorithm force_backup) {
  REASSERT(not simplex.empty(), "Empty simplex passed in!");
  constexpr uint64_t TWO = 2U;

  const uint64_t num_vertices = simplex.size();
  const uint64_t num_subsets = pow(TWO, num_vertices);

  // Initialize a matrix for our matrix cofactors
  DeltaMatrix deltas{DeltaMatrix::Zero(num_subsets - 1, num_vertices)};

  // Populate the base case for the Johnson subalgorithm. Our cofactors for
  // single-element cases are all set to one.
  populate_base_cases(num_vertices, InOut{deltas});

  // Compute the closest subset to the origin as a bitset.
  const bool only_populate_deltas = force_backup == testing::Algorithm::BACKUP;
  const uint64_t result_subset = find_closest_subset(
      simplex,
      num_subsets,
      only_populate_deltas,
      InOut{deltas});

  // Use the backup procedure if necessary
  if (result_subset == FAILURE_SENTINEL) {
    return backup_distance_procedure(simplex, num_subsets, deltas);
  }

  return DistanceResult<DIM>{
      .closest_point = compute_closest_point(simplex, result_subset, deltas),
      .simplex = compute_new_simplex(simplex, result_subset),
  };
}

template DistanceResult<2> distance_subalgorithm(
    const Simplex<2> &simplex,
    testing::Algorithm force_backup);
template DistanceResult<3> distance_subalgorithm(
    const Simplex<3> &simplex,
    testing::Algorithm force_backup);

}  // namespace resim::geometry::convex_helpers
