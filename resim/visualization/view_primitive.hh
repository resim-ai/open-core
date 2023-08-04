// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <optional>
#include <string>
#include <variant>

#include "resim/actor/state/trajectory.hh"
#include "resim/curves/d_curve.hh"
#include "resim/curves/t_curve.hh"
#include "resim/transforms/frame.hh"
#include "resim/transforms/framed_vector.hh"
#include "resim/transforms/se3.hh"
#include "resim/transforms/so3.hh"
#include "resim/utils/uuid.hh"

namespace resim::visualization {

// A struct representing a single visualizable piece of data (e.g. a transform,
// curve, box, etc.), its optional, user-supplied name and metadata about that
// view object.
struct ViewPrimitive {
  UUID id;
  std::variant<
      transforms::SE3,
      transforms::SO3,
      curves::DCurve<transforms::SE3>,
      curves::TCurve<transforms::SE3>,
      actor::state::Trajectory,
      transforms::Frame<3>,
      transforms::FramedVector<3>>
      payload;
  std::optional<std::string> user_defined_name = {};
  // The file name and line number where this view primitive was created.
  std::string file_name;
  int line_number;
};

}  // namespace resim::visualization
