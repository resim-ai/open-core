#pragma once

#include <vector>

#include "resim/curves/d_curve.hh"
#include "resim/transforms/frame.hh"
#include "resim/transforms/liegroup_concepts.hh"

namespace resim::curves {

template <transforms::LieGroupType Group>
class DCurveCircle {
 public:
  using Frame = transforms::Frame<Group::DIMS>;
  inline static const Frame REF_FRAME = Frame::new_frame();
  inline static const Frame PNT_FRAME = Frame::new_frame();

  // Builds and returns a vector of Group that creates a unit circle.
  static std::vector<Group> points(
      const Frame &ref_frame = REF_FRAME,
      const Frame &pnt_frame = PNT_FRAME);
};

}  // namespace resim::curves
