#include "transforms/so3.hh"

namespace resim {
namespace transforms {

SO3::SO3(const Eigen::AngleAxisd &angle_axis) :
    rotation_matrix_(angle_axis.toRotationMatrix()) {}

const Eigen::Matrix3d &SO3::rotation_matrix() const { return rotation_matrix_; }

} // namespace transforms
} // namespace resim

