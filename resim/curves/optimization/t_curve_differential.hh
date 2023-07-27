
#pragma once

#include "resim/curves/optimization/two_jet_tangent_space.hh"
#include "resim/curves/t_curve.hh"
#include "resim/curves/two_jet.hh"
#include "resim/transforms/liegroup_concepts.hh"

namespace resim::curves::optimization {

// A struct holding the differential of TCurve::point_at() with respect to the
// previous and next points in the curve using the two jet tangent spaces
// defined in two_jet_tangent_space.hh. More explicitly, d_prev and d_next are
// matrices such that for all vectors v in the two jet tangent space:
//
// lim h->0 [difference(point_at(time, accumulate(prev, hv), next),
//                      point_at(time, prev, next)) / h] == d_prev * v
// And:
//
// lim h->0 [difference(point_at(time, prev, accumulate(next, hv)),
//                      point_at(time, prev, next)) / h] == d_next * v
//
template <transforms::LieGroupType Group>
struct TCurvePointWithDifferential {
  TwoJetL<Group> point;
  TwoJetTangentMapping<Group> d_prev;
  TwoJetTangentMapping<Group> d_next;
};

// This function computes TCurve::point_at() while also computing its
// differential with respect to the previous and next control points as
// described above. This duplicates some content from TCurve, but is a free
// function to keep from adding complexity to that class which is not necessary
// for most of its use cases.
// @param[in] time - The time to query at.
// @param[in] prev - The previous control point.
// @param[in] next - The next control point.
// @param[out] differential - The differential as described above.
template <transforms::LieGroupType Group>
TCurvePointWithDifferential<Group> point_at(
    double time,
    const typename TCurve<Group>::Control &prev,
    const typename TCurve<Group>::Control &next);

}  // namespace resim::curves::optimization
