// Copyright 2024 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/planning/drone/control.hh"

#include "resim/math/vector_partition.hh"

namespace resim::planning::drone {

using math::get_block;

Control operator+(const Control &u, const typename Control::Vec &du) {
  return Control{
      .angular_acceleration =
          u.angular_acceleration +
          get_block<Control::Partition, Control::ANGULAR_ACCELERATION>(du),
      .thrust =
          u.thrust + get_block<Control::Partition, Control::THRUST>(du).x(),
  };
}

typename Control::Vec operator-(const Control &u, const Control &v) {
  Control::Vec result;
  get_block<Control::Partition, Control::ANGULAR_ACCELERATION>(result) =
      u.angular_acceleration - v.angular_acceleration;
  get_block<Control::Partition, Control::THRUST>(result).x() =
      u.thrust - v.thrust;
  return result;
}

}  // namespace resim::planning::drone
