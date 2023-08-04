// Copyright 2023 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/metrics/proto/simple_metric.pb.h"
#include "resim/metrics/simple_metric.hh"

namespace resim::metrics::proto {

void pack(const metrics::SimpleMetric &in, metrics::proto::SimpleMetric *out);

metrics::SimpleMetric unpack(const metrics::proto::SimpleMetric &in);

}  // namespace resim::metrics::proto
