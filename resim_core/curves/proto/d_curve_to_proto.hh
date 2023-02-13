#pragma once

#include "resim_core/curves/d_curve.hh"
#include "resim_core/utils/inout.hh"

// Pack and Unpack DCurve into their corresponding protobuf messages.

// Note that these templated packing helpers are intended to
// be used by the explicit pack unpack overloads. As helpers they do not follow
// the standard pattern of top-level packers/unpackers and are therefore not
// called pack/unpack.
namespace resim::curves::proto {

// Pack a DCurve object into corresponding proto message.
// @param[in]  in  - DCurve object to be packed.
// @param[out] out - Pointer to proto message representing DCurve.
template <typename Group, typename Msg>
void pack_d_curve(const DCurve<Group> &in, Msg *out);

// Unpack a DCurve proto message into corresponding DCurve.
// @param[in]       in  - Proto message representing the DCurve.
// @param[in-out]   out - Unpacked DCurve<Group> object.
template <typename Group, typename Msg>
void unpack_d_curve(const Msg &in, InOut<DCurve<Group>> out);

}  // namespace resim::curves::proto
