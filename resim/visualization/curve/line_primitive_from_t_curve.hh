
#pragma once

#include <foxglove/LinePrimitive.pb.h>

#include <chrono>

#include "resim/curves/t_curve.hh"
#include "resim/visualization/color.hh"
#include "resim/visualization/curve/line_primitive_options.hh"

namespace resim::visualization::curve {

// This function samples a given TCurve into the given line primitive with
// options governing the appearence of the resulting LinePrimitive and how
// frequently the curve is sampled to produce the resulting polyline.
// @param[in] curve - The curve to sample.
// @param[in] line - The line primitive to add this information to.
// @param[in] sample_period - The maximum time between adjacent samples.
// @param[in] options - The appearance options to use as described above.
// @throws if line is nullptr.
void line_primitive_from_t_curve(
    const curves::TCurve<transforms::SE3> &curve,
    ::foxglove::LinePrimitive *line,
    double sample_period,
    const LinePrimitiveOptions &options = LinePrimitiveOptions{});

}  // namespace resim::visualization::curve
