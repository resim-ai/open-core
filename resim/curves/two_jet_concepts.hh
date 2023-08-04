// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <concepts>

#include "resim/transforms/liegroup_concepts.hh"

namespace resim::curves {

// There are two TwoJet classes, TwoJetL and TwoJetR and we desire to enforce
// a partially common interface, whereby both twojets implement the methods
// defined below and have a template type parameter that is a valid Lie group
// type.

// clang-format off
template <typename T>
concept TwoJetType =
    transforms::has_composition<T>  &&
    transforms::has_identity<T>     &&
    transforms::has_inverse<T>      &&
    transforms::has_is_approx<T>    &&
    transforms::LieGroupType<typename T::GroupType>;
// clang-format on  

}  // namespace resim::curves
