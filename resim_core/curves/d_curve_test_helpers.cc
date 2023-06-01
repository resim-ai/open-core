#include "resim_core/curves/d_curve_test_helpers.hh"

#include <gtest/gtest.h>

#include "resim_core/assert/assert.hh"
#include "resim_core/curves/d_curve.hh"
#include "resim_core/transforms/se3.hh"

namespace resim::curves {

namespace {
using SE3 = transforms::SE3;
using SO3 = transforms::SO3;
}  // namespace

template <>
std::vector<SE3> DCurveCircle<SE3>::points(
    const Frame &ref_frame,
    const Frame &pnt_frame) {
  std::vector<SE3> points;
  const SE3 ref_from_000deg(Eigen::Vector3d::UnitX(), ref_frame, pnt_frame);
  points.push_back(ref_from_000deg);
  const SE3 ref_from_090deg(
      SO3(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ())),
      Eigen::Vector3d::UnitY(),
      ref_frame,
      pnt_frame);
  points.push_back(ref_from_090deg);
  const SE3 ref_from_180deg(
      SO3(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ())),
      -Eigen::Vector3d::UnitX(),
      ref_frame,
      pnt_frame);
  points.push_back(ref_from_180deg);
  const SE3 ref_from_270deg(
      SO3(Eigen::AngleAxisd(3 * M_PI / 2, Eigen::Vector3d::UnitZ())),
      -Eigen::Vector3d::UnitY(),
      ref_frame,
      pnt_frame);
  points.push_back(ref_from_270deg);
  return points;
}

}  // namespace resim::curves
