#pragma once

#include <foxglove/Point3.pb.h>
#include <foxglove/Vector3.pb.h>

#include <Eigen/Dense>

namespace resim::visualization::foxglove {

// Pack a Vector3d into a ::foxglove::Vector3
void pack_into_foxglove(const Eigen::Vector3d &in, ::foxglove::Vector3 *out);

// Pack a Vector3d into a ::foxglove::Point3
void pack_into_foxglove(const Eigen::Vector3d &in, ::foxglove::Point3 *out);

}  // namespace resim::visualization::foxglove
