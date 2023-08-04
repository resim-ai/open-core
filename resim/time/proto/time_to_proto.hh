// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include <google/protobuf/duration.pb.h>
#include <google/protobuf/timestamp.pb.h>

#include "resim/time/timestamp.hh"

namespace resim::time::proto {

// Pack the given timestamp into a protobuf timestamp message.
void pack(time::Timestamp in, google::protobuf::Timestamp* out);

// Pack the given duration into a protobuf duration message.
void pack(time::Duration in, google::protobuf::Duration* out);

// Unpack a protobuf timestamp message into a time::Timestamp.
time::Timestamp unpack(const google::protobuf::Timestamp& in);

// Unpack a protobuf duration message into a time::Duration.
time::Duration unpack(const google::protobuf::Duration& in);

}  // namespace resim::time::proto
