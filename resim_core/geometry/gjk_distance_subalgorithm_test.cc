#include "resim_core/geometry/gjk_distance_subalgorithm.hh"

#include <gtest/gtest.h>

#include <algorithm>
#include <random>
#include <vector>

#include "resim_core/assert/assert.hh"
#include "resim_core/testing/random_matrix.hh"

namespace resim::geometry::convex_helpers {

class DistanceSubalgorithmTest : public ::testing::Test {
 public:
  using Vec2 = Eigen::Matrix<double, 2, 1>;
  using Vec3 = Eigen::Matrix<double, 3, 1>;

  // Generate a random vector of the given type
  template <typename Vector_t>
  Vector_t random_vector() {
    return resim::testing::random_vector<Vector_t>(rng_);
  }

  // Generate a real number between the given lower and upper bound
  double random_real(const double lb, const double ub) {
    std::uniform_real_distribution<double> dist{lb, ub};
    return dist(rng_);
  }

 private:
  static constexpr std::size_t SEED = 9234U;
  std::mt19937 rng_{SEED};
};

// Helper to determine whether a given simplex (assumed to have DIM + 1
// elements) contains the given point within a small tolerance.
template <int DIM>
void expect_in_simplex(
    const Simplex<DIM> &simplex,
    const Eigen::Matrix<double, DIM, 1> &point) {
  ASSERT_EQ(simplex.size(), DIM + 1);
  using EdgeMatrix = Eigen::Matrix<double, DIM, DIM>;
  EdgeMatrix edge_matrix;
  for (int ii = 0; ii < DIM; ++ii) {
    edge_matrix.col(ii) = simplex.at(ii + 1) - simplex.at(0);
  }
  const Eigen::Matrix<double, DIM, 1> barycentric_coordinates{
      edge_matrix.inverse() * (point - simplex.at(0))};

  constexpr double ONE = 1.;
  constexpr double TOLERANCE = 1e-6;
  for (int kk = 0; kk < barycentric_coordinates.rows(); ++kk) {
    EXPECT_LT(-TOLERANCE, barycentric_coordinates(kk));
    EXPECT_LT(barycentric_coordinates(kk), ONE + TOLERANCE);
  }
}

////////////////////////////////////////////////////////////////////////////////
// 2 Dimensional Test Cases
////////////////////////////////////////////////////////////////////////////////

// Test that we fail on an empty simplex passed in
TEST_F(DistanceSubalgorithmTest, TestFailOnEmptySimplex) {
  // SETUP
  constexpr int DIM = 2;
  const Simplex<DIM> simplex{};

  // ACTION / VERIFICATION
  EXPECT_THROW(distance_subalgorithm(simplex), AssertException);
}

// Test that the distance subalgorithm works for single vertices.
TEST_F(DistanceSubalgorithmTest, TestSingleVertex2D) {
  // SETUP
  constexpr int DIM = 2;
  constexpr int NUM_TESTS = 100U;

  for (int ii = 0; ii < NUM_TESTS; ++ii) {
    const Simplex<DIM> simplex{{random_vector<Vec2>()}};

    // ACTION
    const DistanceResult result{distance_subalgorithm(simplex)};

    // VERIFICATION
    EXPECT_EQ(result.simplex, simplex);
    EXPECT_EQ(result.closest_point, simplex.at(0));
  }
}

// Test that the distance subalgorithm works for a line segment where the
// solution is expected to be along the edge.
TEST_F(DistanceSubalgorithmTest, TestLineSegment2DEdgeSolution) {
  // SETUP
  constexpr int DIM = 2;
  constexpr int NUM_TESTS = 100U;

  for (int ii = 0; ii < NUM_TESTS; ++ii) {
    for (const testing::Algorithm force :
         {testing::Algorithm::BACKUP, testing::Algorithm::MAIN}) {
      const Vec2 random_point{random_vector<Vec2>()};
      const Vec2 perpendicular_dir{-random_point.y(), random_point.x()};

      constexpr double LB = 0.1;
      constexpr double UB = 1.0;

      // SETUP
      Simplex<DIM> simplex{};
      simplex.push_back(random_point + random_real(LB, UB) * perpendicular_dir);
      simplex.push_back(random_point - random_real(LB, UB) * perpendicular_dir);

      // ACTION
      const DistanceResult result{distance_subalgorithm(simplex, force)};

      // VERIFICATION
      EXPECT_EQ(result.simplex, simplex);
      EXPECT_TRUE(result.closest_point.isApprox(random_point));
    }
  }
}

// Test that the distance subalgorithm works for a line segment where the
// solution is expected to be on one of the vertices.
TEST_F(DistanceSubalgorithmTest, TestLineSegment2DVertexSolution) {
  // SETUP
  constexpr int DIM = 2;
  constexpr int NUM_TESTS = 100U;

  for (int ii = 0; ii < NUM_TESTS; ++ii) {
    const Vec2 random_point{random_vector<Vec2>()};

    constexpr double LB = 0.1;
    constexpr double UB = 1.0;

    // SETUP
    Simplex<DIM> simplex{};
    simplex.push_back(random_point * (1. - random_real(LB, UB)));
    simplex.push_back(random_point * (1. + random_real(LB, UB)));

    // ACTION
    const DistanceResult result{distance_subalgorithm(simplex)};

    // VERIFICATION
    const Simplex<DIM> expected_simplex{simplex.at(0)};
    EXPECT_EQ(result.simplex, expected_simplex);
    EXPECT_TRUE(result.closest_point.isApprox(simplex.at(0)));
  }
}

// Test a bunch of random triangles and verify that the resulting closest points
// are inside them and at least as close as all vertices
TEST_F(DistanceSubalgorithmTest, TestRandomTriangle2D) {
  // SETUP
  constexpr int DIM = 2;
  constexpr int NUM_TESTS = 100;
  for (int ii = 0; ii < NUM_TESTS; ++ii) {
    Simplex<DIM> simplex;
    constexpr int NUM_VERTICES = 3;
    for (int jj = 0; jj < NUM_VERTICES; ++jj) {
      simplex.push_back(random_vector<Vec2>());
    }

    // ACTION
    const DistanceResult result{distance_subalgorithm(simplex)};

    // VERIFICATION
    expect_in_simplex(simplex, result.closest_point);

    for (const Vec2 &vertex : simplex) {
      EXPECT_LE(result.closest_point.norm(), vertex.norm());
    }
  }
}

TEST_F(DistanceSubalgorithmTest, TestRandomTriangle2DForced) {
  // SETUP
  constexpr int DIM = 2;
  constexpr int NUM_TESTS = 100;
  for (int ii = 0; ii < NUM_TESTS; ++ii) {
    Simplex<DIM> simplex;
    constexpr int NUM_VERTICES = 3;
    for (int jj = 0; jj < NUM_VERTICES; ++jj) {
      simplex.push_back(random_vector<Vec2>());
    }

    // ACTION
    const DistanceResult result{
        distance_subalgorithm(simplex, testing::Algorithm::BACKUP)};

    // VERIFICATION
    expect_in_simplex(simplex, result.closest_point);
    for (const Vec2 &vertex : simplex) {
      EXPECT_LE(result.closest_point.norm(), vertex.norm());
    }
  }
}

// Test a triangle which contains the origin
TEST_F(DistanceSubalgorithmTest, TestTriangle2DContainsOrigin) {
  // SETUP
  constexpr int DIM = 2;
  const Simplex<DIM> simplex{
      {1., 1.},
      {-1., 1.},
      {0., -1.},
  };
  const Vec2 expected_closest_point{Vec2::Zero()};

  // ACTION
  for (const testing::Algorithm force :
       {testing::Algorithm::BACKUP, testing::Algorithm::MAIN}) {
    const DistanceResult result{distance_subalgorithm(simplex, force)};

    // VERIFICATION
    EXPECT_EQ(result.simplex, simplex);
    EXPECT_TRUE(result.closest_point.isApprox(expected_closest_point));
  }
}

// Test a triangle where we expect an edge to be the closest subsimplex
TEST_F(DistanceSubalgorithmTest, TestTriangle2DEdgeClosest) {
  // SETUP
  constexpr int DIM = 2;
  const Simplex<DIM> simplex{
      {1., 1.},
      {-1., 1.},
      {0., 2.},
  };
  const Vec2 expected_closest_point{Vec2::UnitY()};
  const Simplex<DIM> expected_simplex{
      simplex.at(0),
      simplex.at(1),
  };

  // ACTION
  for (const testing::Algorithm force :
       {testing::Algorithm::BACKUP, testing::Algorithm::MAIN}) {
    const DistanceResult result{distance_subalgorithm(simplex, force)};

    // VERIFICATION
    EXPECT_EQ(result.simplex, expected_simplex);
    EXPECT_TRUE(result.closest_point.isApprox(expected_closest_point));
  }
}

// Test a triangle where we expect a vertex to be the closest subsimplex
TEST_F(DistanceSubalgorithmTest, TestTriangle2DVertexClosest) {
  // SETUP
  constexpr int DIM = 2;
  const Simplex<DIM> simplex{
      {0., 1.},
      {1., 2.},
      {-1., 2.},
  };
  const Vec2 expected_closest_point{Vec2::UnitY()};
  const Simplex<DIM> expected_simplex{simplex.at(0)};

  // ACTION
  for (const testing::Algorithm force :
       {testing::Algorithm::BACKUP, testing::Algorithm::MAIN}) {
    const DistanceResult result{distance_subalgorithm(simplex, force)};

    // VERIFICATION
    EXPECT_EQ(result.simplex, expected_simplex);
    EXPECT_TRUE(result.closest_point.isApprox(expected_closest_point));
  }
}

////////////////////////////////////////////////////////////////////////////////
// 3 Dimensional Test Cases
////////////////////////////////////////////////////////////////////////////////

// Test a bunch of random tetrahedra and verify that the resulting closest
// points are inside them and at least as close as all vertices
TEST_F(DistanceSubalgorithmTest, TestRandomTetrahedron3D) {
  // SETUP
  constexpr int DIM = 3;
  constexpr int NUM_TESTS = 100;
  for (int ii = 0; ii < NUM_TESTS; ++ii) {
    Simplex<DIM> simplex;
    constexpr int NUM_VERTICES = 4;
    for (int jj = 0; jj < NUM_VERTICES; ++jj) {
      simplex.push_back(random_vector<Vec3>());
    }

    // ACTION
    const DistanceResult result{distance_subalgorithm(simplex)};

    // VERIFICATION
    expect_in_simplex(simplex, result.closest_point);
    for (const Vec3 &vertex : simplex) {
      EXPECT_LE(result.closest_point.norm(), vertex.norm());
    }
  }
}

TEST_F(DistanceSubalgorithmTest, TestRandomTetrahedron3DForced) {
  // SETUP
  constexpr int DIM = 3;

  constexpr int NUM_TESTS = 100;
  for (int ii = 0; ii < NUM_TESTS; ++ii) {
    Simplex<DIM> simplex;
    constexpr int NUM_VERTICES = 4;
    for (int jj = 0; jj < NUM_VERTICES; ++jj) {
      simplex.push_back(random_vector<Vec3>());
    }

    // ACTION
    const DistanceResult result{
        distance_subalgorithm(simplex, testing::Algorithm::BACKUP)};

    // VERIFICATION
    expect_in_simplex(simplex, result.closest_point);
    for (const Vec3 &vertex : simplex) {
      EXPECT_LE(result.closest_point.norm(), vertex.norm());
    }
  }
}

// Test a triangle centered above the origin whose normal points in the z
// direction.
TEST_F(DistanceSubalgorithmTest, TestTriangle3DFaceClosest) {
  // SETUP
  constexpr int DIM = 3;
  const Simplex<DIM> simplex{
      {1., 0., 1.},
      {-1., 1., 1.},
      {-1., -1., 1.},
  };
  const Vec3 expected_closest_point{Vec3::UnitZ()};

  // ACTION
  const DistanceResult result{distance_subalgorithm(simplex)};

  // VERIFICATION
  EXPECT_EQ(result.simplex, simplex);
  EXPECT_TRUE(result.closest_point.isApprox(expected_closest_point));
}

// Test a triangle where we expect one of the sides to result from the
// distance subalgorithm.
TEST_F(DistanceSubalgorithmTest, TestTriangle3DEdgeClosest) {
  // SETUP
  constexpr int DIM = 3;
  const Simplex<DIM> simplex{
      {1., 0., 1.},
      {-1., 0., 1.},
      {0., 0., 2.},
  };
  const Vec3 expected_closest_point{Vec3::UnitZ()};
  const Simplex<DIM> expected_simplex{simplex.at(0), simplex.at(1)};

  // ACTION
  const DistanceResult result{distance_subalgorithm(simplex)};

  // VERIFICATION
  EXPECT_EQ(result.simplex, expected_simplex);
  EXPECT_TRUE(result.closest_point.isApprox(expected_closest_point));
}

// Test a triangle where we expect one of the vertices to result
// from the distance subalgorithm.
TEST_F(DistanceSubalgorithmTest, TestTriangle3DVertexClosest) {
  // SETUP
  constexpr int DIM = 3;
  const Simplex<DIM> simplex{
      {1., 0., 2.},
      {-1., 0., 2.},
      {0., 0., 1.},
  };
  const Vec3 expected_closest_point{Vec3::UnitZ()};
  const Simplex<DIM> expected_simplex{simplex.at(2)};

  // ACTION
  const DistanceResult result{distance_subalgorithm(simplex)};

  // VERIFICATION
  EXPECT_EQ(result.simplex, expected_simplex);
  EXPECT_TRUE(result.closest_point.isApprox(expected_closest_point));
}

// Test a tetrahedron which contains the origin
TEST_F(DistanceSubalgorithmTest, TestTetrahedron3dContainsOrigin) {
  // SETUP
  constexpr int DIM = 3;
  const Simplex<DIM> simplex{
      {1., 0., 1.},
      {-1., 0., 1.},
      {0., 1., -1.},
      {0., -1., -1.},
  };
  const Vec3 expected_closest_point{Vec3::Zero()};

  // ACTION
  const DistanceResult result{distance_subalgorithm(simplex)};

  // VERIFICATION
  EXPECT_EQ(result.simplex, simplex);
  EXPECT_TRUE(result.closest_point.isApprox(expected_closest_point));
}

// Test a tetrahedron where we expect a triangle to be the closest subsimplex
TEST_F(DistanceSubalgorithmTest, TestTetrahedron3dFaceClosest) {
  // SETUP
  constexpr int DIM = 3;

  const Simplex<DIM> simplex{
      {1., 0., 1.},
      {-1., 1., 1.},
      {-1., -1., 1.},
      {0., 0., 2.},
  };
  const Vec3 expected_closest_point{Vec3::UnitZ()};
  const Simplex<DIM> expected_simplex{
      simplex.at(0),
      simplex.at(1),
      simplex.at(2)};

  // ACTION
  const DistanceResult result{distance_subalgorithm(simplex)};

  // VERIFICATION
  EXPECT_EQ(result.simplex, expected_simplex);
  EXPECT_TRUE(result.closest_point.isApprox(expected_closest_point));
}

// Test a tetrahedron where we expect an edge to be the closest subsimplex
TEST_F(DistanceSubalgorithmTest, TestTetrahedron3dEdgeClosest) {
  // SETUP
  constexpr int DIM = 3;
  const Simplex<DIM> simplex{
      {1., 0., 1.},
      {-1., 0., 1.},
      {0., -1., 2.},
      {0., 1., 2.},
  };
  const Vec3 expected_closest_point{Vec3::UnitZ()};
  const Simplex<DIM> expected_simplex{simplex.at(0), simplex.at(1)};

  // ACTION
  const DistanceResult result{distance_subalgorithm(simplex)};

  // VERIFICATION
  EXPECT_EQ(result.simplex, expected_simplex);
  EXPECT_TRUE(result.closest_point.isApprox(expected_closest_point));
}

// Test a tetrahedron where we expect a vertex to be the closest subsimplex
TEST_F(DistanceSubalgorithmTest, TestTetrahedron3dVertexClosest) {
  // SETUP
  constexpr int DIM = 3;
  const Simplex<DIM> simplex{
      {0., 0., 1.},
      {1., 0., 2.},
      {-1., 1., 2.},
      {-1., -1., 2.},
  };
  const Vec3 expected_closest_point{Vec3::UnitZ()};
  const Simplex<DIM> expected_simplex{simplex.at(0)};

  // ACTION
  const DistanceResult result{distance_subalgorithm(simplex)};

  // VERIFICATION
  EXPECT_EQ(result.simplex, expected_simplex);
  EXPECT_TRUE(result.closest_point.isApprox(expected_closest_point));
}

// Test for a single case that the algorithm gives the same results for a bunch
// of different permutations.
TEST_F(DistanceSubalgorithmTest, TestTetrahedron3dPermutations) {
  constexpr int DIM = 3;

  // Test a tetrahedron where we expect a vertex to be the closest subsimplex
  {
    // SETUP
    const Simplex<DIM> base_simplex{
        {1., 0., 1.},
        {-1., 0., 1.},
        {0., -1., 2.},
        {0., 1., 2.},
    };
    const Vec3 expected_closest_point{Vec3::UnitZ()};

    std::vector<std::size_t> indices{0U, 1U, 2U, 3U};
    do {
      const Simplex<DIM> simplex{
          base_simplex.at(indices.at(0U)),
          base_simplex.at(indices.at(1U)),
          base_simplex.at(indices.at(2U)),
          base_simplex.at(indices.at(3U)),
      };

      // ACTION
      const DistanceResult result{distance_subalgorithm(simplex)};

      // VERIFICATION
      constexpr std::size_t EDGE_SIZE = 2U;
      EXPECT_EQ(result.simplex.size(), EDGE_SIZE);
      for (std::size_t ii = 0; ii < EDGE_SIZE; ++ii) {
        EXPECT_NE(
            std::find(
                result.simplex.cbegin(),
                result.simplex.cend(),
                base_simplex.at(ii)),
            result.simplex.cend());
      }
      EXPECT_TRUE(result.closest_point.isApprox(expected_closest_point));
    } while (std::next_permutation(indices.begin(), indices.end()));
  }
}

}  // namespace resim::geometry::convex_helpers
