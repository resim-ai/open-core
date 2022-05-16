#include "transforms/se3.hh"

namespace resim {
namespace transforms {

using TangentVector = SE3::TangentVector;

SE3::SE3(const SO3 &rotation) : 
    rotation_(rotation), 
    translation_(Eigen::Vector3d::Zero()) {}

SE3::SE3(const Eigen::Vector3d &translation) : 
    rotation_(SO3::identity()), 
    translation_(translation) {}

SE3::SE3(const SO3 &rotation, const Eigen::Vector3d &translation) : 
    rotation_(rotation), 
    translation_(translation) {}

const SO3 &SE3::rotation() const { return rotation_; }

const Eigen::Vector3d &SE3::translation() const { return translation_; }

} // namespace transforms
} // namespace resim
