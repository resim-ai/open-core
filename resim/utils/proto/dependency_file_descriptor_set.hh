// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <google/protobuf/descriptor.pb.h>

namespace resim {
//
// This function returns the file descriptor set of dependencies as a serialized
// string. This is needed for creating schemas through the MCAP C++ API. As
// described here:
// https://github.com/foxglove/mcap/blob/main/docs/specification/appendix.md We
// need to attach a serialized binary of the relevant protobuf message type's
// file descriptor set as the `data` attribute of the new schema when creating
// it.
// @param[in] root - The descriptor for the message whose dependency file
//                   descriptor set we would like to generate.
std::string dependency_file_descriptor_set(
    const google::protobuf::Descriptor &root);

}  // namespace resim
