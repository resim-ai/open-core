
#include "resim/visualization/log/visit_topic.hh"

#include "resim/assert/assert.hh"
#include "resim/utils/inout.hh"

namespace resim::visualization::log {

void visit_topic(
    const std::string_view topic,
    InOut<mcap::McapReader> reader,
    const std::function<void(const mcap::MessageView &)> &visitor) {
  mcap::ReadMessageOptions read_options;
  read_options.topicFilter = [&topic](const std::string_view t) {
    return t == topic;
  };
  const mcap::ProblemCallback on_problem = [](const mcap::Status &s) {
    REASSERT(false, "MCAP Read Error: " + s.message);
  };
  for (const mcap::MessageView &view :
       reader->readMessages(on_problem, read_options)) {
    visitor(view);
  }
}

}  // namespace resim::visualization::log
