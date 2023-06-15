#pragma once

#include <vector>

#include "resim/visualization/proto/view_object_metadata.pb.h"
#include "resim/visualization/view_primitive.hh"

namespace resim::visualization::proto {

// Pack a ViewPrimitive into a ViewObjectMetadata proto. We pass through the
// appropriate name, since the ViewPrimitive itself doesn't know the name if it
// hasn't been supplied by the user i.e. default name using resim::view <<
// object syntax
void pack_metadata(
    const visualization::ViewPrimitive &in,
    const std::string &name,
    ViewObjectMetadata *out);

// Pack a vector of ViewObjectMetadata into a ViewObjectMetadataList proto.
void pack_metadata_list(
    const std::vector<ViewObjectMetadata> &in,
    ViewObjectMetadataList *out);

// Note: we do not unpack ViewObjectMetadata (or ViewObjectMetadata list) into
// ViewPrimitive, because they are a lossy representation of the original
// ViewPrimitive, since the actual payload is omitted.
//
// The reason we do this right now is to avoid the size overhead of passing
// around potentially large payloads. We will revist this decision should we
// wish to allow the user to view the raw data on the View UI.
//
//

};  // namespace resim::visualization::proto
