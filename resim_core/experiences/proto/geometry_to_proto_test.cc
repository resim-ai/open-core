#include "resim_core/experiences/proto/geometry_to_proto.hh"

#include <google/protobuf/util/message_differencer.h>
#include <gtest/gtest.h>

#include <variant>

#include "resim_core/assert/assert.hh"
#include "resim_core/experiences/geometry.hh"
#include "resim_core/experiences/proto/geometry.pb.h"
#include "resim_core/geometry/drone_wireframe.hh"
#include "resim_core/geometry/proto/wireframe_to_proto.hh"
#include "resim_core/geometry/wireframe.hh"
#include "resim_core/utils/uuid.hh"

namespace resim::experiences {

namespace {

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

}  // namespace

TEST(GeometryToProtoTest, TestPack) {
  // SETUP
  const geometry::Wireframe wireframe{geometry::drone_wireframe(extents)};
  const Geometry test_geometry{
      .id = UUID::new_uuid(),
      .model = wireframe,
  };

  // ACTION
  proto::Geometry message;
  pack(test_geometry, &message);

  // VERIFICATION
  geometry::proto::Wireframe expected_wireframe;
  pack(wireframe, &expected_wireframe);
  EXPECT_EQ(message.id().data(), test_geometry.id.to_string());
  ASSERT_EQ(message.model_case(), proto::Geometry::kWireframe);
  EXPECT_PRED2(
      google::protobuf::util::MessageDifferencer::Equals,
      message.wireframe(),
      expected_wireframe);
}

TEST(GeometryToProtoTest, TestUnpack) {
  // SETUP
  const geometry::Wireframe wireframe{geometry::drone_wireframe(extents)};
  const Geometry test_geometry{
      .id = UUID::new_uuid(),
      .model = wireframe,
  };
  proto::Geometry message;
  pack(test_geometry, &message);

  // ACTION
  const Geometry unpacked{unpack(message)};

  // VERIFICATION
  EXPECT_EQ(test_geometry.id, unpacked.id);
  const bool has_wireframe =
      std::holds_alternative<geometry::Wireframe>(unpacked.model);
  ASSERT_TRUE(has_wireframe);
  const geometry::Wireframe unpacked_wireframe{
      std::get<geometry::Wireframe>(unpacked.model)};

  EXPECT_EQ(wireframe.points(), unpacked_wireframe.points());
  EXPECT_EQ(wireframe.edges(), unpacked_wireframe.edges());
}

TEST(GeometryToProtoTest, TestUnpackUnset) {
  // SETUP
  const geometry::Wireframe wireframe{geometry::drone_wireframe(extents)};
  const Geometry test_geometry{
      .id = UUID::new_uuid(),
      .model = wireframe,
  };
  proto::Geometry message;
  pack(test_geometry, &message);
  message.clear_model();

  // ACTION / VERIFICATION
  EXPECT_THROW(unpack(message), AssertException);
}

}  // namespace resim::experiences
