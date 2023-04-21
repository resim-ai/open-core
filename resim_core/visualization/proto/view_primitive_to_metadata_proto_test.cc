#include "resim_core/visualization/proto/view_primitive_to_metadata_proto.hh"

#include <gtest/gtest.h>

#include "resim_core/actor/state/trajectory.hh"
#include "resim_core/curves/d_curve.hh"
#include "resim_core/curves/t_curve.hh"
#include "resim_core/transforms/frame.hh"
#include "resim_core/transforms/framed_group.hh"
#include "resim_core/transforms/se3.hh"
#include "resim_core/transforms/so3.hh"
#include "resim_core/utils/match.hh"
#include "resim_core/visualization/proto/view_object_metadata.pb.h"
#include "resim_core/visualization/proto/view_primitive_test_helper.hh"
#include "resim_core/visualization/view_primitive.hh"

namespace resim::visualization {

namespace {
using transforms::FSE3;
using transforms::FSO3;
using transforms::SE3;
using transforms::SO3;
using Frame = transforms::Frame<3>;
constexpr int NUMBER_TESTS = 3;

const std::array<std::string, NUMBER_TESTS> NAME_RANGE = {
    "first_name",
    "second_name",
    "third_name"};

}  // namespace

template <typename T>
class ViewPrimitiveToMetadataProtoTypedTest : public ::testing::Test {};

using PayloadTypes = ::testing::Types<
    Frame,
    SE3,
    SO3,
    FSE3,
    FSO3,
    curves::DCurve<SE3>,
    curves::DCurve<FSE3>,
    curves::TCurve<FSE3>,
    actor::state::Trajectory>;

TYPED_TEST_SUITE(ViewPrimitiveToMetadataProtoTypedTest, PayloadTypes);

// NOLINTBEGIN(readability-function-cognitive-complexity)
TYPED_TEST(ViewPrimitiveToMetadataProtoTypedTest, TestPack) {
  for (const auto &name : NAME_RANGE) {
    // SETUP
    const ViewPrimitive test_primitive =
        generate_test_primitive<TypeParam>(name);
    proto::ViewObjectMetadata metadata_proto;
    // ACTION
    pack_metadata(test_primitive, name, &metadata_proto);
    // VERIFICATION
    EXPECT_EQ(metadata_proto.name(), name);
    EXPECT_EQ(
        metadata_proto.file(),
        resim::visualization::detail::TEST_FILE_NAME);
    EXPECT_EQ(metadata_proto.line_number(), detail::TEST_LINE_NUMBER);
    // Now check the enum:
    match(
        test_primitive.payload,
        [&](const Frame &test_frame) {
          EXPECT_EQ(metadata_proto.type(), proto::ReSimType::TYPE_FRAME);
        },
        [&](const SE3 &test_se3) {
          EXPECT_EQ(metadata_proto.type(), proto::ReSimType::TYPE_SE3);
        },
        [&](const SO3 &test_so3) {
          EXPECT_EQ(metadata_proto.type(), proto::ReSimType::TYPE_SO3);
        },
        [&](const FSE3 &test_fse3) {
          EXPECT_EQ(metadata_proto.type(), proto::ReSimType::TYPE_FSE3);
        },
        [&](const FSO3 &test_fso3) {
          EXPECT_EQ(metadata_proto.type(), proto::ReSimType::TYPE_FSO3);
        },
        [&](const curves::DCurve<SE3> &test_d_curve_se3) {
          EXPECT_EQ(metadata_proto.type(), proto::ReSimType::TYPE_DCURVE_SE3);
        },
        [&](const curves::DCurve<FSE3> &test_d_curve_fse3) {
          EXPECT_EQ(metadata_proto.type(), proto::ReSimType::TYPE_DCURVE_FSE3);
        },
        [&](const curves::TCurve<FSE3> &test_t_curve) {
          EXPECT_EQ(metadata_proto.type(), proto::ReSimType::TYPE_TCURVE_FSE3);
        },
        [&](const actor::state::Trajectory &test_trajectory) {
          EXPECT_EQ(metadata_proto.type(), proto::ReSimType::TYPE_TRAJECTORY);
        });
  }
}
// NOLINTEND(readability-function-cognitive-complexity)

TYPED_TEST(ViewPrimitiveToMetadataProtoTypedTest, TestPackInvalid) {
  ViewPrimitive test_primitive = generate_test_primitive<TypeParam>(
      NAME_RANGE.at(0));  // We use the first name

  // ACTION/VERIFICATION
  EXPECT_THROW(
      proto::pack_metadata(test_primitive, NAME_RANGE.at(0), nullptr),
      AssertException);
}

// NOLINTBEGIN(readability-function-cognitive-complexity)
TYPED_TEST(ViewPrimitiveToMetadataProtoTypedTest, TestListPack) {
  // SETUP
  const ViewPrimitive test_primitive_a =
      generate_test_primitive<TypeParam>(NAME_RANGE.at(0));
  const ViewPrimitive test_primitive_b =
      generate_test_primitive<TypeParam>(NAME_RANGE.at(1));
  const ViewPrimitive test_primitive_c =
      generate_test_primitive<TypeParam>(NAME_RANGE.at(2));
  proto::ViewObjectMetadata metadata_proto_a;
  proto::ViewObjectMetadata metadata_proto_b;
  proto::ViewObjectMetadata metadata_proto_c;
  const std::vector<ViewPrimitive> primitives = {
      test_primitive_a,
      test_primitive_b,
      test_primitive_c};
  std::vector<proto::ViewObjectMetadata> metadata = {
      metadata_proto_a,
      metadata_proto_b,
      metadata_proto_c};
  // ACTION
  for (int i = 0; i < NUMBER_TESTS; i++) {
    pack_metadata(primitives.at(i), NAME_RANGE.at(i), &metadata.at(i));
  }
  proto::ViewObjectMetadataList metadata_list_proto;
  pack_metadata_list(metadata, &metadata_list_proto);
  // VERIFICATION
  ASSERT_EQ(metadata_list_proto.metadata_size(), NUMBER_TESTS);
  for (int i = 0; i < NUMBER_TESTS; ++i) {
    EXPECT_EQ(metadata_list_proto.metadata(i).name(), NAME_RANGE.at(i));
    EXPECT_EQ(
        metadata_list_proto.metadata(i).file(),
        resim::visualization::detail::TEST_FILE_NAME);
    EXPECT_EQ(
        metadata_list_proto.metadata(i).line_number(),
        detail::TEST_LINE_NUMBER);
    // Now check the enum:
    match(
        primitives.at(i).payload,
        [&](const Frame &test_frame) {
          EXPECT_EQ(metadata.at(i).type(), proto::ReSimType::TYPE_FRAME);
        },
        [&](const SE3 &test_se3) {
          EXPECT_EQ(metadata.at(i).type(), proto::ReSimType::TYPE_SE3);
        },
        [&](const SO3 &test_so3) {
          EXPECT_EQ(metadata.at(i).type(), proto::ReSimType::TYPE_SO3);
        },
        [&](const FSE3 &test_fse3) {
          EXPECT_EQ(metadata.at(i).type(), proto::ReSimType::TYPE_FSE3);
        },
        [&](const FSO3 &test_fso3) {
          EXPECT_EQ(metadata.at(i).type(), proto::ReSimType::TYPE_FSO3);
        },
        [&](const curves::DCurve<SE3> &test_d_curve_se3) {
          EXPECT_EQ(metadata.at(i).type(), proto::ReSimType::TYPE_DCURVE_SE3);
        },
        [&](const curves::DCurve<FSE3> &test_d_curve_fse3) {
          EXPECT_EQ(metadata.at(i).type(), proto::ReSimType::TYPE_DCURVE_FSE3);
        },
        [&](const curves::TCurve<FSE3> &test_t_curve) {
          EXPECT_EQ(metadata.at(i).type(), proto::ReSimType::TYPE_TCURVE_FSE3);
        },
        [&](const actor::state::Trajectory &test_trajectory) {
          EXPECT_EQ(metadata.at(i).type(), proto::ReSimType::TYPE_TRAJECTORY);
        });
  }
}
// NOLINTEND(readability-function-cognitive-complexity)

TYPED_TEST(ViewPrimitiveToMetadataProtoTypedTest, TestPackListInvalid) {
  ViewPrimitive test_primitive = generate_test_primitive<TypeParam>(
      std::nullopt);  // We use the default name
  proto::ViewObjectMetadata test_metadata;
  pack_metadata(test_primitive, NAME_RANGE.at(0), &test_metadata);
  std::vector<proto::ViewObjectMetadata> test_metadata_list = {test_metadata};
  // ACTION/VERIFICATION
  EXPECT_THROW(
      proto::pack_metadata_list(test_metadata_list, nullptr),
      AssertException);
}

}  // namespace resim::visualization
