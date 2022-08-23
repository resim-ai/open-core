#pragma once

#include <Eigen/Dense>

namespace resim::transforms {

// Cross product in matrix form.
// cross_matrix arranges the three dimensional vector vec in matrix form, such
// that cross_matrix(vec) * other_vec is equivalent to vec.cross(other_vec).
Eigen::Matrix3d cross_matrix(const Eigen::Vector3d &vec);

}  // namespace resim::transforms
