// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <google/protobuf/repeated_field.h>

#include <Eigen/Dense>
#include <algorithm>

#include "resim/assert/assert.hh"
#include "resim/utils/inout.hh"

// Pack and Unpack Eigen Matrices into a protobuf repeated field of doubles.
// This works with any Eigen Matrix type that is one or two dimensional and
// has element type double. It may work with other matrix types, but this is not
// tested. Both static and dynamic sized matrices are supported.

// Note that these specific packers and unpackers break the general pattern used
// for packers and unpackers elsewhere in the ReSim codebase. Specifically:
// A. They are not named "pack" and "unpack".
// B. The Unpacker uses an InOut arg to fill the Matix, not a return.
// There are three reasons for this.
// 1. These pack a field not a message so will generally be called within an
//    existing pack/unpack function that is working on a message.
// 2. The matrix type is templated and the Proto field type is very common; we
//    seek to avoid any overload resolution issues.
// 3. Change 'B' is specifically to support dynamic Matrices, who's dimensions
//    are unknown until they are initialized with size.
namespace resim::math::proto {

// Pack an Eigen Matrix into a repeated field double.
// @param[in]  in  - The Eigen Matrix to be packed.
// @param[out] out - A pointer to a repeated field double.
// Note the '.reshaped()' method in Eigen Matrix objects exists specifically
// to provide a one-dimensional view of a matrix. By default it orders the
// elements in a column-major fashion. However the ordering is unimportant as
// long as the user does not try to use some other method to pack/unpack.
template <typename MatrixType>
void pack_matrix(
    const MatrixType &in,
    google::protobuf::RepeatedField<double> *const out) {
  REASSERT(out != nullptr, "Can't pack into invalid proto!");
  out->Clear();
  const auto flat_in = in.reshaped();
  out->Add(flat_in.cbegin(), flat_in.cend());
}

// Unpack an Eigen Matrix into a repeated field double.
// @param[in]  in  - The proto repeated field double to be unpacked.
// @param[in-out] out - An Eigen matrix who's size is known.
// Note the Eigen matrix needs to be instantiated before calling this function
// and - in particular - dynamic matrices need to be initialized with size so
// that the expected size can be checked against the size of the repeated field.
template <typename MatrixType>
void unpack_matrix(
    const google::protobuf::RepeatedField<double> &in,
    InOut<MatrixType> out) {
  constexpr auto SIZE_ERR =
      "The expected number of elements in the Matrix (as indicated by .size()) "
      "must match the number of elements in the proto field.";
  REASSERT(in.size() == out->size(), SIZE_ERR);
  std::copy(in.cbegin(), in.cend(), out->reshaped().begin());
}

}  // namespace resim::math::proto
