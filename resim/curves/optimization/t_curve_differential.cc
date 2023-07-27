
#include "resim/curves/optimization/t_curve_differential.hh"

#include "resim/assert/assert.hh"
#include "resim/curves/optimization/two_jet_tangent_space.hh"
#include "resim/curves/quintic_poly_coeffs.hh"
#include "resim/curves/two_jet.hh"
#include "resim/math/vector_partition.hh"
#include "resim/transforms/frame.hh"
#include "resim/transforms/se3.hh"
#include "resim/transforms/so3.hh"

namespace resim::curves::optimization {

namespace {

// A helper type representing block indices to be used with auto lambdas to
// easily query matrix blocks. See the lambda get_prev and get_next defined in
// the point_at() function below.
template <int R, int C>
struct Block {
  static constexpr int ROW = R;
  static constexpr int COL = C;
};

}  // namespace

// The implementation of this function is based off of the contents of this
// document: https://ethaneade.com/lie_spline.pdf
// The notation from this document is often borrowed.
template <transforms::LieGroupType Group>
TCurvePointWithDifferential<Group> point_at(
    const double time,
    const typename TCurve<Group>::Control &prev,
    const typename TCurve<Group>::Control &next) {
  using TwoJet = TwoJetL<Group>;
  using Frame = transforms::Frame<Group::DIMS>;

  const Frame into{next.point.frame_from_ref().into()};
  const Frame from{next.point.frame_from_ref().from()};
  REASSERT(
      prev.point.frame_from_ref().verify_frames(into, from),
      "Control points must have the same ref and point frame!");

  REASSERT(
      time >= prev.time and time <= next.time,
      "Query time out of bounds!");

  const double time_nrm = (time - prev.time) / (next.time - prev.time);
  const double dt = (next.time - prev.time);

  // Normalized origin and destination two jets:
  const TwoJet orig{
      prev.point.frame_from_ref(),
      dt * prev.point.d_frame_from_ref(),
      dt * dt * prev.point.d2_frame_from_ref()};

  const TwoJet dest{
      next.point.frame_from_ref(),
      dt * next.point.d_frame_from_ref(),
      dt * dt * next.point.d2_frame_from_ref()};

  // Compute the primitive curve S(t) first:
  constexpr int QUINTIC = 5;
  using TangentVector = typename Group::TangentVector;
  using TangentMapping = typename Group::TangentMapping;

  // Set up our recursive computation of S(t):
  // Inputs:
  // Coefficients (the a_i(t) from the document referenced above).
  std::array<TwoJetPolyCoeffs, QUINTIC> coeff{
      QuinticPolyCoeffs::dest_from_orig(time_nrm),
      QuinticPolyCoeffs::d_orig(time_nrm),
      QuinticPolyCoeffs::d_dest(time_nrm),
      QuinticPolyCoeffs::d2_orig(time_nrm),
      QuinticPolyCoeffs::d2_dest(time_nrm),
  };

  const Group dest_from_orig =
      dest.frame_from_ref() * orig.frame_from_ref().inverse();

  // Algebra values (the p_i(t) from the document referenced above)
  std::array<TangentVector, QUINTIC> p{
      dest_from_orig.log(),
      orig.d_frame_from_ref(),
      dest.d_frame_from_ref(),
      orig.d2_frame_from_ref(),
      dest.d2_frame_from_ref(),
  };

  // Outputs:
  // The point we accumulate the value of S_i(t) in recursively:
  TwoJet point{TwoJet::identity(into, into)};

  // The independent terms (A_i = exp(a_i p_i)) along with their derivatives.
  std::array<TwoJet, QUINTIC> A;

  // Partial results that we need to compute differentials later
  std::array<TangentVector, QUINTIC> dpoint;
  std::array<TangentVector, QUINTIC> d2point;

  // Compute the S(t) (i.e. point) recursively:
  for (int ii = 0; ii < QUINTIC; ++ii) {
    A.at(ii) = TwoJet{
        Group::exp(coeff.at(ii).a * p.at(ii), into, into),
        coeff.at(ii).da * p.at(ii),
        coeff.at(ii).d2a * p.at(ii)};

    point = A.at(ii) * point;

    dpoint.at(ii) = point.d_frame_from_ref();
    d2point.at(ii) = point.d2_frame_from_ref();
  }

  // Begin the backwards recursion procedure from the document above (the
  // section called "Structural Differentials for Regression") to compute the
  // differentials of S(t).
  Group L{Group::identity()};
  TangentVector l{TangentVector::Zero()};

  using GroupMat = Eigen::Matrix<double, Group::DOF, Group::DOF>;

  // Outputs:
  std::array<GroupMat, QUINTIC> dS_dp;
  std::array<GroupMat, QUINTIC> dS_dot_dp;
  std::array<GroupMat, QUINTIC> dS_dot_dot_dp;
  for (int ii = QUINTIC - 1; ii >= 0; --ii) {
    // d(S_i) / dp_i
    const GroupMat dSi_dp{
        Group::exp_diff(coeff.at(ii).a * p.at(ii)) * coeff.at(ii).a};

    // d(S_i') / dp_i base case
    GroupMat dSi_dot_dp{coeff.at(ii).da * GroupMat::Identity()};

    // d(S_i'') / dp_i base case
    GroupMat dSi_dot_dot_dp{coeff.at(ii).d2a * GroupMat::Identity()};

    // Non-base terms for d(S_i') / dp_i and d(S_i'') / dp_i
    if (ii > 0) {
      const GroupMat A_ii_adjoint{A.at(ii).frame_from_ref().adjoint()};
      dSi_dot_dp -= Group::adjoint(A_ii_adjoint * dpoint.at(ii - 1)) * dSi_dp;
      dSi_dot_dot_dp +=
          -(Group::adjoint(A_ii_adjoint * d2point.at(ii - 1)) +
            Group::adjoint(A.at(ii).d_frame_from_ref()) *
                Group::adjoint(A_ii_adjoint * dpoint.at(ii - 1))) *
              dSi_dp -
          coeff.at(ii).da * Group::adjoint(A_ii_adjoint * dpoint.at(ii - 1));
    }
    // Compute the differentials:
    dS_dp.at(ii) = L.adjoint() * dSi_dp;
    dS_dot_dp.at(ii) = L.adjoint() * dSi_dot_dp;
    dS_dot_dot_dp.at(ii) =
        L.adjoint() * dSi_dot_dot_dp + Group::adjoint(l) * dS_dot_dp.at(ii);

    // Recurse:
    l = l + L.adjoint() * A.at(ii).d_frame_from_ref();
    L = L * A.at(ii).frame_from_ref();
  }

  // Re-map these differentials of S(t) to B(t) = S(t) * B_0 using the
  // expressions in the document.
  const TangentMapping log_diff{
      Group::exp_diff(dest_from_orig.log()).inverse()};
  const GroupMat &dp0_ddest{log_diff};
  const GroupMat dp0_dorig{log_diff * (-dest_from_orig.adjoint())};

  TCurvePointWithDifferential<Group> result{};

  // Set up helper lambdas to make accessing d_prev and d_next easy
  const auto get_prev = [&d_prev = result.d_prev](auto block) {
    constexpr int ROW = decltype(block)::ROW;
    constexpr int COL = decltype(block)::COL;
    return math::
        get_block<TwoJetPartition<Group>, ROW, TwoJetPartition<Group>, COL>(
            d_prev);
  };
  const auto get_next = [&d_next = result.d_next](auto block) {
    constexpr int ROW = decltype(block)::ROW;
    constexpr int COL = decltype(block)::COL;
    return math::
        get_block<TwoJetPartition<Group>, ROW, TwoJetPartition<Group>, COL>(
            d_next);
  };

  // Get the indices from the two jet partition:
  constexpr int POINT = FRAME_FROM_REF;
  constexpr int DPOINT = D_FRAME_FROM_REF;
  constexpr int D2POINT = D2_FRAME_FROM_REF;

  get_prev(Block<FRAME_FROM_REF, FRAME_FROM_REF>()) =
      dS_dp.at(0) * dp0_dorig + point.frame_from_ref().adjoint();
  get_prev(Block<POINT, DPOINT>()) = dS_dp.at(1) * dt;
  get_prev(Block<POINT, D2POINT>()) = dS_dp.at(3) * dt * dt;
  get_prev(Block<DPOINT, POINT>()) = dS_dot_dp.at(0) * dp0_dorig / dt;
  get_prev(Block<DPOINT, DPOINT>()) = dS_dot_dp.at(1);
  get_prev(Block<DPOINT, D2POINT>()) = dS_dot_dp.at(3) * dt;
  get_prev(Block<D2POINT, POINT>()) = dS_dot_dot_dp.at(0) * dp0_dorig / dt / dt;
  get_prev(Block<D2POINT, DPOINT>()) = dS_dot_dot_dp.at(1) / dt;
  get_prev(Block<D2POINT, D2POINT>()) = dS_dot_dot_dp.at(3);

  get_next(Block<POINT, POINT>()) = dS_dp.at(0) * dp0_ddest;
  get_next(Block<POINT, DPOINT>()) = dS_dp.at(2) * dt;
  get_next(Block<POINT, D2POINT>()) = dS_dp.at(4) * dt * dt;
  get_next(Block<DPOINT, POINT>()) = dS_dot_dp.at(0) * dp0_ddest / dt;
  get_next(Block<DPOINT, DPOINT>()) = dS_dot_dp.at(2);
  get_next(Block<DPOINT, D2POINT>()) = dS_dot_dp.at(4) * dt;
  get_next(Block<D2POINT, POINT>()) = dS_dot_dot_dp.at(0) * dp0_ddest / dt / dt;
  get_next(Block<D2POINT, DPOINT>()) = dS_dot_dot_dp.at(2) / dt;
  get_next(Block<D2POINT, D2POINT>()) = dS_dot_dot_dp.at(4);

  Group frame_from_ref{point.frame_from_ref() * orig.frame_from_ref()};
  point.set_frame_from_ref(frame_from_ref);
  point.set_d_frame_from_ref(point.d_frame_from_ref() / dt);
  point.set_d2_frame_from_ref(point.d2_frame_from_ref() / dt / dt);
  result.point = point;
  return result;
}

template TCurvePointWithDifferential<transforms::SO3> point_at<transforms::SO3>(
    double time,
    const typename TCurve<transforms::SO3>::Control &prev,
    const typename TCurve<transforms::SO3>::Control &next);

template TCurvePointWithDifferential<transforms::SE3> point_at<transforms::SE3>(
    double time,
    const typename TCurve<transforms::SE3>::Control &prev,
    const typename TCurve<transforms::SE3>::Control &next);

}  // namespace resim::curves::optimization
