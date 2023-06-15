#pragma once

#include "resim/curves/two_jet.hh"
#include "resim/utils/inout.hh"

// Pack and Unpack TwoJetL and TwoJetR into their corresponding protobuf
// messages. For example:
// - TwoJetL<SE3> -> proto::TwoJetL_SE3
// - TwoJetL<SO3> -> proto::TwoJetL_SO3.
//
// Note that these templated packing helpers are intended to
// be used by the explicit pack unpack overloads. As helpers they do not follow
// the standard pattern of top-level packers/unpackers and are therefore not
// called pack/unpack.
namespace resim::curves::proto {

// Pack a TwoJetL object into a corresponding proto message.
// @param[in]  in  - The TwoJetL object to be packed.
// @param[out] out - A pointer to a proto message representing the
//                   TwoJetL.
template <typename Group, typename Msg>
void pack_two_jetl(const TwoJetL<Group> &in, Msg *out);

// Unpack a TwoJetL proto message into a corresponding TwoJetL.
// @param[in]  in  - A proto message representing the TwoJetL.
// @param[in-out] out - An unpacked TwoJetL<Group> object.
template <typename Group, typename Msg>
void unpack_two_jetl(const Msg &in, InOut<TwoJetL<Group>> out);

// Pack a TwoJetR object into a corresponding proto message.
// @param[in]  in  - The TwoJetR object to be packed.
// @param[out] out - A pointer to a proto message representing the
//                   TwoJetR.
template <typename Group, typename Msg>
void pack_two_jetr(const TwoJetR<Group> &in, Msg *out);

// Unpack a TwoJetR proto message into a corresponding TwoJetR.
// @param[in]  in  - A proto message representing the TwoJetR.
// @param[in-out] out - An unpacked TwoJetR<Group> object.
template <typename Group, typename Msg>
void unpack_two_jetr(const Msg &in, InOut<TwoJetR<Group>> out);
}  // namespace resim::curves::proto
