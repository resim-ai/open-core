#include "resim/visualization/proto/view_primitive_test_helper.hh"

#include <gtest/gtest.h>

#include <optional>
#include <random>
#include <string>

#include "resim/actor/state/trajectory.hh"
#include "resim/curves/d_curve.hh"
#include "resim/curves/t_curve.hh"
#include "resim/transforms/frame.hh"
#include "resim/transforms/framed_vector.hh"
#include "resim/transforms/se3.hh"
#include "resim/transforms/so3.hh"
#include "resim/utils/match.hh"
#include "resim/visualization/view_primitive.hh"

namespace resim::visualization {

namespace {
using transforms::SE3;
using transforms::SO3;
using Frame = transforms::Frame<3>;
using FramedVector = transforms::FramedVector<3>;
constexpr auto TEST_NAME = "test_name";
}  // namespace

template <typename T>
class ViewPrimitiveTestHelperTest : public ::testing::Test {};

using PayloadTypes = ::testing::Types<
    Frame,
    SE3,
    SO3,
    curves::DCurve<SE3>,
    curves::TCurve<SE3>,
    actor::state::Trajectory>;

TYPED_TEST_SUITE(ViewPrimitiveTestHelperTest, PayloadTypes);

// NOLINTBEGIN(readability-function-cognitive-complexity)
TYPED_TEST(ViewPrimitiveTestHelperTest, TestGenerateTestPrimitive) {
  // SETUP/ACTION
  const ViewPrimitive test_primitive =
      generate_test_primitive<TypeParam>(TEST_NAME);
  // VERIFICATION
  EXPECT_TRUE(test_primitive.user_defined_name.has_value());
  EXPECT_EQ(test_primitive.user_defined_name.value(), TEST_NAME);
  EXPECT_EQ(test_primitive.file_name, detail::TEST_FILE_NAME);
  EXPECT_EQ(test_primitive.line_number, detail::TEST_LINE_NUMBER);
  match(
      test_primitive.payload,
      [&](const Frame &test_frame) {
        // Non-deterministic generation of frames, so we can't test for equality
      },
      [&](const SE3 &test_se3) {
        EXPECT_TRUE(test_se3.is_approx_transform(generate_test_object<SE3>()));
      },
      [&](const SO3 &test_so3) {
        EXPECT_TRUE(test_so3.is_approx_transform(generate_test_object<SO3>()));
      },
      [&](const curves::DCurve<SE3> &test_dcurve_se3) {
        const curves::DCurve<SE3> generated_d_curve_se3{
            generate_test_object<curves::DCurve<SE3>>()};
        EXPECT_EQ(
            test_dcurve_se3.control_pts().size(),
            generated_d_curve_se3.control_pts().size());
        for (int i = 0; i < test_dcurve_se3.control_pts().size(); i++) {
          EXPECT_TRUE(
              test_dcurve_se3.control_pts().at(i).ref_from_control->is_approx(
                  *generated_d_curve_se3.control_pts().at(i).ref_from_control));
          EXPECT_EQ(
              test_dcurve_se3.control_pts().at(i).arc_length,
              generated_d_curve_se3.control_pts().at(i).arc_length);
        }
      },
      [&](const curves::TCurve<SE3> &test_tcurve_se3) {
        const curves::TCurve<SE3> generated_t_curve_se3{
            generate_test_object<curves::TCurve<SE3>>()};
        EXPECT_EQ(
            test_tcurve_se3.control_pts().size(),
            generated_t_curve_se3.control_pts().size());
        for (int i = 0; i < test_tcurve_se3.control_pts().size(); i++) {
          EXPECT_TRUE(
              test_tcurve_se3.control_pts()
                  .at(i)
                  .point.frame_from_ref()
                  .is_approx_transform(generated_t_curve_se3.control_pts()
                                           .at(i)
                                           .point.frame_from_ref()));
          EXPECT_EQ(
              test_tcurve_se3.control_pts().at(i).point.d_frame_from_ref(),
              generated_t_curve_se3.control_pts()
                  .at(i)
                  .point.d_frame_from_ref());
          EXPECT_EQ(
              test_tcurve_se3.control_pts().at(i).point.d2_frame_from_ref(),
              generated_t_curve_se3.control_pts()
                  .at(i)
                  .point.d2_frame_from_ref());
          EXPECT_DOUBLE_EQ(
              test_tcurve_se3.control_pts().at(i).time,
              generated_t_curve_se3.control_pts().at(i).time);
        }
      },
      [&](const actor::state::Trajectory &test_trajectory) {
        const auto &test_tcurve_se3 = test_trajectory.curve();
        const auto &generated_trajectory =
            generate_test_object<actor::state::Trajectory>();
        const curves::TCurve<SE3> generated_t_curve_se3 =
            generated_trajectory.curve();
        EXPECT_EQ(
            test_trajectory.start_time(),
            generated_trajectory.start_time());
        EXPECT_EQ(
            test_tcurve_se3.control_pts().size(),
            generated_t_curve_se3.control_pts().size());
        for (int i = 0; i < test_tcurve_se3.control_pts().size(); i++) {
          EXPECT_TRUE(
              test_tcurve_se3.control_pts()
                  .at(i)
                  .point.frame_from_ref()
                  .is_approx_transform(generated_t_curve_se3.control_pts()
                                           .at(i)
                                           .point.frame_from_ref()));

          EXPECT_EQ(
              test_tcurve_se3.control_pts().at(i).point.d_frame_from_ref(),
              generated_t_curve_se3.control_pts()
                  .at(i)
                  .point.d_frame_from_ref());
          EXPECT_EQ(
              test_tcurve_se3.control_pts().at(i).point.d2_frame_from_ref(),
              generated_t_curve_se3.control_pts()
                  .at(i)
                  .point.d2_frame_from_ref());
          EXPECT_DOUBLE_EQ(
              test_tcurve_se3.control_pts().at(i).time,
              generated_t_curve_se3.control_pts().at(i).time);
        }
      },
      [&](const FramedVector &test_framed_vector) {});
}
// NOLINTEND(readability-function-cognitive-complexity)
}  // namespace resim::visualization
