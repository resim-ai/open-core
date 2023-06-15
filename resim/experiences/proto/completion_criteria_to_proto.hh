#pragma once

#include "resim/experiences/completion_criteria.hh"
#include "resim/experiences/proto/completion_criteria.pb.h"

namespace resim::experiences::proto {

void pack(const experiences::CompletionCriteria &in, CompletionCriteria *out);

experiences::CompletionCriteria unpack(const CompletionCriteria &in);

}  // namespace resim::experiences::proto
