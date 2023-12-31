// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include "resim/transforms/liegroup_concepts.hh"
#include "resim/utils/inout.hh"

// Pack and Unpack Liegroups into their corresponding protobuf messages.
// For example SO3 -> proto::SO3, SE3 -> proto::SE3.
// Note that these templated packing helpers are intended to be used by
// the explicit pack unpack overloads. As helpers they do not follow the
// standard pattern of top-level packers/unpackers and are therefore not called
// pack/unpack.
namespace resim::transforms::proto {

// Pack a LieGroupType object into a corresponding proto message.
// @param[in]  in  - The LieGroupType object to be packed.
// @param[out] out - A pointer to a proto message representing the LieGroupType.
template <transforms::LieGroupType Group, typename Msg>
void pack_liegroup(const Group &in, Msg *out);

// Unpack a LieGroup proto message into a corresponding LieGroupType.
// @param[in]  in  - A proto message representing the LieGroupType.
// @param[in-out] out - An unpacked LieGroupType object.
template <transforms::LieGroupType Group, typename Msg>
void unpack_liegroup(const Msg &in, InOut<Group> out);

}  // namespace resim::transforms::proto
