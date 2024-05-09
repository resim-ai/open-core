// Copyright 2024 ReSim, Inc.
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "resim/visualization/log/extract_experience.hh"

#include <optional>

#include "resim/experiences/proto/experience.pb.h"
#include "resim/experiences/proto/experience_to_proto.hh"
#include "resim/visualization/log/visit_topic.hh"

namespace resim::visualization::log {

experiences::Experience extract_experience(InOut<mcap::McapReader> reader) {
  std::optional<experiences::Experience> experience;

  visit_topic(
      "/experience",
      reader,
      [&experience](const mcap::MessageView &view) {
        experiences::proto::Experience experience_msg;
        REASSERT(experience_msg.ParseFromArray(
            view.message.data,
            view.message.dataSize));
        REASSERT(
            not experience.has_value(),
            "More than one experience found in input log!");
        experience = unpack(experience_msg);
      });
  REASSERT(experience.has_value(), "No experience found in input log!");
  return *experience;
}

}  // namespace resim::visualization::log
