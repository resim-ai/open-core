#include "resim_core/experiences/proto/experience_to_proto.hh"

#include "resim_core/assert/assert.hh"
#include "resim_core/experiences/experience.hh"
#include "resim_core/experiences/proto/dynamic_behavior_to_proto.hh"
#include "resim_core/experiences/proto/experience.pb.h"

namespace resim::experiences::proto {

void pack(const experiences::Experience &in, Experience *const out) {
  REASSERT(out != nullptr, "Can't pack into invalid proto!");
  out->Clear();
  Experience_Header *const header = out->mutable_header();
  header->set_name(in.header.name);
  header->set_parent_experience_name(in.header.parent_experience_name);
  header->set_description(in.header.description);
  Revision *const revision = header->mutable_experience_schema_revision();
  revision->set_major_revision(in.header.revision.major);
  revision->set_minor_revision(in.header.revision.minor);
  pack(in.dynamic_behavior, out->mutable_dynamic_behavior());
}

experiences::Experience unpack(const Experience &in) {
  experiences::Experience result;
  result.dynamic_behavior = unpack(in.dynamic_behavior());
  experiences::Revision revision{
      .major = in.header().experience_schema_revision().major_revision(),
      .minor = in.header().experience_schema_revision().minor_revision(),
  };
  experiences::Header header{
      .revision = revision,
      .name = in.header().name(),
      .description = in.header().description(),
      .parent_experience_name = in.header().parent_experience_name(),
  };
  result.header = header;
  return result;
}

}  // namespace resim::experiences::proto
