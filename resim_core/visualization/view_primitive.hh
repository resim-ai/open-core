
#pragma once

#include <optional>
#include <string>
#include <variant>

#include "resim_core/actor/state/trajectory.hh"
#include "resim_core/curves/d_curve.hh"
#include "resim_core/curves/t_curve.hh"
#include "resim_core/transforms/frame.hh"
#include "resim_core/transforms/framed_group.hh"
#include "resim_core/transforms/framed_vector.hh"
#include "resim_core/transforms/se3.hh"
#include "resim_core/transforms/so3.hh"
#include "resim_core/utils/uuid.hh"

namespace resim::visualization {

// A struct representing a single visualizable piece of data (e.g. a transform,
// curve, box, etc.), its optional, user-supplied name and metadata about that
// view object.
struct ViewPrimitive {
  UUID id;
  std::variant<
      transforms::SE3,
      transforms::SO3,
      transforms::FSE3,
      transforms::FSO3,
      curves::DCurve<transforms::SE3>,
      curves::DCurve<transforms::FSE3>,
      curves::TCurve<transforms::FSE3>,
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
