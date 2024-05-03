
#include "resim/visualization/log/test_helpers.hh"

#include "resim/experiences/actor.hh"
#include "resim/experiences/experience.hh"
#include "resim/experiences/geometry.hh"
#include "resim/geometry/drone_wireframe.hh"
#include "resim/geometry/wireframe.hh"
#include "resim/utils/mcap_logger.hh"
#include "resim/utils/uuid.hh"

namespace resim::visualization::log {

namespace {

geometry::Wireframe test_wireframe() {
  constexpr double CHASSIS_RADIUS_M = 1.;
  constexpr double ROTOR_LATERAL_OFFSET_M = 0.3;
  constexpr double ROTOR_VERTICAL_OFFSET_M = 0.3;
  constexpr double ROTOR_RADIUS_M = 0.5;
  constexpr std::size_t SAMPLES_PER_ROTOR = 2;

  const geometry::DroneExtents extents{
      .chassis_radius_m = CHASSIS_RADIUS_M,
      .rotor_lateral_offset_m = ROTOR_LATERAL_OFFSET_M,
      .rotor_vertical_offset_m = ROTOR_VERTICAL_OFFSET_M,
      .rotor_radius_m = ROTOR_RADIUS_M,
      .samples_per_rotor = SAMPLES_PER_ROTOR,
  };
  return geometry::drone_wireframe(extents);
}

}  // namespace

using experiences::Experience;

Experience test_experience() {
  Experience experience;

  // Setup the header
  experience.header.revision.major = 1;
  experience.header.revision.minor = 2;
  experience.header.name = "test_experience";
  experience.header.description = "An experience for testing.";
  experience.header.parent_experience_name = "";

  // Add geometry
  const experiences::Geometry geometry{
      .id = UUID::new_uuid(),
      .model = test_wireframe(),
  };
  experience.geometries.emplace(geometry.id, geometry);

  // Add an actor
  const experiences::GeometryReference geometry_ref{
      .geometry_id = geometry.id,
  };
  const experiences::Actor actor{
      .id = UUID::new_uuid(),
      .name = "test_actor",
      .actor_type = experiences::ActorType::SYSTEM_UNDER_TEST,
      .geometries = {geometry_ref},
  };
  experience.dynamic_behavior.actors.emplace_back(actor);

  // The storyboard and completion criteria are not needed yet for
  // visualization.

  return experience;
}

std::string make_input_log(
    const experiences::proto::Experience& experience_msg,
    const int num_messages) {
  std::ostringstream sstream;
  // Scope so that the logger flushes to the string stream when it's destroyed.
  {
    McapLogger logger{sstream};

    logger.add_proto_channel<experiences::proto::Experience>("/experience");

    constexpr time::Timestamp TIME;
    for (int ii = 0; ii < num_messages; ++ii) {
      logger.log_proto("/experience", TIME, experience_msg);
    }
  }
  return sstream.str();
}

StreamReader::StreamReader(std::istream& stream) : stream_{stream} {
  stream_.seekg(0, std::istream::end);
  size_ = stream_.tellg();
  stream_.seekg(0, std::istream::beg);
}

uint64_t StreamReader::size() const { return size_; }
uint64_t
StreamReader::read(std::byte** output, uint64_t offset, uint64_t size) {
  if (offset >= size_) {
    return 0;
  }

  if (offset != position_) {
    stream_.seekg(static_cast<std::streamoff>(offset));
    position_ = offset;
  }

  if (size > buffer_.size()) {
    buffer_.resize(size);
  }

  // We need to use memcpy to avoid a pointer cast, which clang-tidy doesn't
  // like:
  std::string tmp(size, '\0');
  stream_.read(tmp.data(), static_cast<std::streamoff>(size));
  std::memcpy(buffer_.data(), tmp.data(), size);
  *output = buffer_.data();

  const uint64_t bytesRead = stream_.gcount();
  position_ += bytesRead;
  return bytesRead;
}

}  // namespace resim::visualization::log
