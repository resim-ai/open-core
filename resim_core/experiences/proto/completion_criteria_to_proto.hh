#pragma once

#include "resim_core/experiences/completion_criteria.hh"
#include "resim_core/experiences/proto/completion_criteria.pb.h"

namespace resim::experiences::proto {

void pack(const experiences::CompletionCriteria &in, CompletionCriteria *out);

experiences::CompletionCriteria unpack(const CompletionCriteria &in);

}  // namespace resim::experiences::proto
