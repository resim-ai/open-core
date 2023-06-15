#pragma once

#include "resim/curves/t_curve.hh"
#include "resim/utils/inout.hh"

// Pack and Unpack TCurves into their corresponding protobuf messages.
// For example:
// - TCurve<SE3> -> proto::TCurve_SE3
// - TCurve<SO3> -> proto::TCurve_SO3.
//
// Note that these templated packing helpers are intended to
// be used by the explicit pack unpack overloads. As helpers they do not follow
// the standard pattern of top-level packers/unpackers and are therefore not
// called pack/unpack.
namespace resim::curves::proto {

// Pack a TCurve object into a corresponding proto message.
// @param[in]  in  - The TCurve object to be packed.
// @param[out] out - A pointer to a proto message representing the
//                   TCurve.
template <typename Group, typename Msg>
void pack_t_curve(const TCurve<Group> &in, Msg *out);

// Unpack a TCurve proto message into a corresponding TCurve.
// @param[in]  in  - A proto message representing the TCurve.
// @param[in-out] out - An unpacked TCurve<Group> object.
template <typename Group, typename Msg>
void unpack_t_curve(const Msg &in, InOut<TCurve<Group>> out);

}  // namespace resim::curves::proto
