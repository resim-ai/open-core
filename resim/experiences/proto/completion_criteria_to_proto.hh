// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#pragma once

#include "resim/experiences/completion_criteria.hh"
#include "resim/experiences/proto/completion_criteria.pb.h"

namespace resim::experiences::proto {

void pack(const experiences::CompletionCriteria &in, CompletionCriteria *out);

experiences::CompletionCriteria unpack(const CompletionCriteria &in);

}  // namespace resim::experiences::proto
