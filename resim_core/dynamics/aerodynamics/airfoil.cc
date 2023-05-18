#include "resim_core/dynamics/aerodynamics/airfoil.hh"

#include <Eigen/Dense>

#include "resim_core/assert/assert.hh"
#include "resim_core/dynamics/aerodynamics/rigid_body_aerodynamics.hh"
#include "resim_core/transforms/frame.hh"
#include "resim_core/transforms/framed_group.hh"
#include "resim_core/transforms/framed_vector.hh"

namespace resim::dynamics::aerodynamics {

constexpr double TWO = 2.0;
constexpr double FOUR = 4.0;

AirfoilElementConfig::AirfoilElementConfig(
    double lift_curve_slope_coeff_,
    double skin_friction_coeff_,
    double zero_lift_angle_rad_,
    double stall_angle_pos_rad_,
    double stall_angle_neg_rad_,
    double total_chord_m_,
    double flap_chord_m_,
    double span_m_,
    AirfoilFlapType flap_type_)
    : lift_curve_slope_coeff(lift_curve_slope_coeff_),
      skin_friction_coeff(skin_friction_coeff_),
      zero_lift_angle_rad(zero_lift_angle_rad_),
      stall_angle_pos_rad(stall_angle_pos_rad_),
      stall_angle_neg_rad(stall_angle_neg_rad_),
      total_chord_m(total_chord_m_),
      flap_chord_m(flap_chord_m_),
      span_m(span_m_),
      flap_type(flap_type_),
      area_m_squared(total_chord_m * span_m),
      aspect_ratio(span_m / total_chord_m),
      adjusted_lift_curve_slope_coeff(
          lift_curve_slope_coeff_ * aspect_ratio /
          (aspect_ratio + TWO * (aspect_ratio + FOUR) / (aspect_ratio + TWO))),
      theta_f_rad(std::acos(TWO * (flap_chord_m / total_chord_m) - 1.0)),
      flap_effectiveness_tau_coeff(
          1.0 - ((theta_f_rad - std::sin(theta_f_rad)) / M_PI)),
      lift_coeff_max_pos(
          adjusted_lift_curve_slope_coeff *
          (stall_angle_pos_rad_ - zero_lift_angle_rad_)),
      lift_coeff_max_neg(
          adjusted_lift_curve_slope_coeff *
          (stall_angle_neg_rad_ - zero_lift_angle_rad_)) {
  REASSERT(total_chord_m_ > 0.0);
  REASSERT(flap_chord_m_ >= 0.0);
  REASSERT(flap_chord_m_ <= total_chord_m_);

  REASSERT(span_m_ > 0.0);
  REASSERT(stall_angle_pos_rad_ >= 0.0);
  REASSERT(stall_angle_neg_rad_ <= 0.0);
}

AirfoilElement::AirfoilElement(
    transforms::FSE3 com_from_cop,
    AirfoilElementConfig config)
    : AerodynamicElementImpl<AirfoilElement, AirfoilElementState>(
          std::move(com_from_cop)),
      config_(config) {}

AerodynamicElement::FramedVector AirfoilElement::aerodynamics_impl_(
    const AerodynamicElement::FramedVector& cop_local_wind,
    const AirfoilElementState& state) const {
  const double angle_of_attack_rad =
      std::atan(cop_local_wind.z() / cop_local_wind.x());

  const double lift_delta = config_.adjusted_lift_curve_slope_coeff *
                            config_.flap_effectiveness_tau_coeff *
                            viscosity_factor_eta_(state) * state.flap_angle_rad;
  const double adjusted_zero_lift_angle_rad =
      config_.zero_lift_angle_rad -
      (lift_delta / config_.lift_curve_slope_coeff);

  const double delta_lift_coeff_max =
      lift_delta * d_lift_coeff_max_from_lift_coeff_();
  const double adjusted_lift_coeff_max_pos =
      config_.lift_coeff_max_pos + delta_lift_coeff_max;
  const double adjusted_lift_coeff_max_neg =
      config_.lift_coeff_max_neg + delta_lift_coeff_max;
  const double adjusted_stall_angle_pos_rad =
      adjusted_zero_lift_angle_rad +
      adjusted_lift_coeff_max_pos / config_.adjusted_lift_curve_slope_coeff;
  const double adjusted_stall_angle_neg_rad =
      adjusted_zero_lift_angle_rad +
      adjusted_lift_coeff_max_neg / config_.adjusted_lift_curve_slope_coeff;

  // TODO(tknowles): This is a hack that I don't fully understand
  const double padding_pos_rad =
      0.25 - 0.16 * std::clamp(0.5 + 0.57 * state.flap_angle_rad, 0.0, 1.0);
  const double padding_neg_rad =
      0.25 - 0.16 * std::clamp(0.5 - 0.57 * state.flap_angle_rad, 0.0, 1.0);

  DragCoefficients drag_coeffs{};
  if (angle_of_attack_rad < adjusted_stall_angle_pos_rad &&
      angle_of_attack_rad > adjusted_stall_angle_neg_rad) {
    // Low angle-of-attack domain
    drag_coeffs = compute_low_angle_drag_coeffs_(
        angle_of_attack_rad,
        adjusted_zero_lift_angle_rad);
  } else if (
      // High angle-of-attack domain
      angle_of_attack_rad > adjusted_stall_angle_pos_rad + padding_pos_rad ||
      angle_of_attack_rad < adjusted_stall_angle_neg_rad - padding_neg_rad) {
    drag_coeffs = compute_beyond_stall_drag_coeffs_(
        state,
        angle_of_attack_rad,
        adjusted_zero_lift_angle_rad,
        adjusted_stall_angle_pos_rad,
        adjusted_stall_angle_neg_rad);
  } else if (
      // Stall angle-of-attack domain (positive)
      angle_of_attack_rad >= adjusted_stall_angle_pos_rad
      // && (angle_of_attack_rad <= adjusted_stall_angle_pos_rad +
      // padding_pos_rad)
  ) {
    const DragCoefficients drag_coeffs_at_stall =
        compute_low_angle_drag_coeffs_(
            adjusted_stall_angle_pos_rad,
            adjusted_zero_lift_angle_rad);
    const DragCoefficients drag_coeffs_beyond_stall =
        compute_beyond_stall_drag_coeffs_(
            state,
            adjusted_stall_angle_pos_rad + padding_pos_rad,
            adjusted_zero_lift_angle_rad,
            adjusted_stall_angle_pos_rad,
            adjusted_stall_angle_neg_rad);
    const double t =
        (angle_of_attack_rad - adjusted_stall_angle_pos_rad) / padding_pos_rad;
    drag_coeffs =
        (1.0 - t) * drag_coeffs_at_stall + t * drag_coeffs_beyond_stall;
  } else {  // if (angle_of_attack_rad <= adjusted_stall_angle_neg_rad &&
    //             angle_of_attack_rad >= adjusted_stall_angle_neg_rad -
    //             padding_neg_rad) {

    // Stall angle-of-attack domain (negative)
    const DragCoefficients drag_coeffs_at_stall =
        compute_low_angle_drag_coeffs_(
            adjusted_stall_angle_neg_rad,
            adjusted_zero_lift_angle_rad);
    const DragCoefficients drag_coeffs_beyond_stall =
        compute_beyond_stall_drag_coeffs_(
            state,
            adjusted_stall_angle_neg_rad - padding_neg_rad,
            adjusted_zero_lift_angle_rad,
            adjusted_stall_angle_pos_rad,
            adjusted_stall_angle_neg_rad);

    const double t =
        -(angle_of_attack_rad - adjusted_stall_angle_neg_rad) / padding_neg_rad;
    drag_coeffs =
        (1.0 - t) * drag_coeffs_at_stall + t * drag_coeffs_beyond_stall;
  }

  const double bernouillis = 0.5 * AIR_DENSITY_KG_PER_M_CUBED *
                             cop_local_wind.squaredNorm() *
                             config_.area_m_squared;

  const double lift = drag_coeffs.lift_coeff() * bernouillis;
  const double drag = drag_coeffs.drag_coeff() * bernouillis;

  return FramedVector(
      lift * cop_local_wind.normalized() +
          drag * cop_local_wind.normalized().cross(Eigen::Vector3d::UnitY()),
      cop_local_wind.frame());
}

DragCoefficients AirfoilElement::compute_low_angle_drag_coeffs_(
    double angle_of_attack_rad,
    double adjusted_zero_lift_angle_rad) const {
  const double lift_coeff =
      config_.adjusted_lift_curve_slope_coeff *
      (angle_of_attack_rad - adjusted_zero_lift_angle_rad);
  const double induced_angle_rad = lift_coeff / (M_PI * config_.aspect_ratio);

  const double effective_angle_rad =
      angle_of_attack_rad - adjusted_zero_lift_angle_rad - induced_angle_rad;

  const double tangent_coeff =
      config_.skin_friction_coeff * std::cos(effective_angle_rad);

  const double normal_coeff =
      (lift_coeff + tangent_coeff * std::sin(effective_angle_rad)) /
      std::cos(effective_angle_rad);
  const double drag_coeff = normal_coeff * std::sin(effective_angle_rad) +
                            tangent_coeff * std::cos(effective_angle_rad);
  const double pitch_coeff =
      normal_coeff_to_pitch_coeff_(normal_coeff, effective_angle_rad);

  return DragCoefficients{lift_coeff, drag_coeff, pitch_coeff};
}

DragCoefficients AirfoilElement::compute_beyond_stall_drag_coeffs_(
    const AirfoilElementState& state,
    double angle_of_attack_rad,
    double adjusted_zero_lift_angle_rad,
    double adjusted_stall_angle_pos_rad,
    double adjusted_stall_angle_neg_rad) const {
  double induced_angle_rad = 0.0;
  if (angle_of_attack_rad >= adjusted_stall_angle_pos_rad) {
    const double adjusted_stall_angle_rad = adjusted_stall_angle_pos_rad;
    const double induced_angle_at_stall_rad =
        (config_.adjusted_lift_curve_slope_coeff *
         (adjusted_stall_angle_rad - adjusted_zero_lift_angle_rad)) /
        (M_PI * config_.aspect_ratio);
    induced_angle_rad =
        induced_angle_at_stall_rad *
        (1.0 - ((angle_of_attack_rad - adjusted_stall_angle_rad) /
                (M_PI_2 - adjusted_stall_angle_rad)));
  } else {
    const double adjusted_stall_angle_rad = adjusted_stall_angle_neg_rad;
    const double induced_angle_at_stall_rad =
        (config_.adjusted_lift_curve_slope_coeff *
         (adjusted_stall_angle_rad - adjusted_zero_lift_angle_rad)) /
        (M_PI * config_.aspect_ratio);
    induced_angle_rad =
        induced_angle_at_stall_rad *
        (1.0 - ((angle_of_attack_rad - adjusted_stall_angle_rad) /
                (-M_PI_2 - adjusted_stall_angle_rad)));
  }

  const double effective_angle_rad =
      angle_of_attack_rad - adjusted_zero_lift_angle_rad - induced_angle_rad;

  const double normal_coeff =
      flat_plate_drag_coeff_(state) * std::sin(effective_angle_rad) *
      ((1.0 / (0.56 + 0.44 * std::abs(std::sin(effective_angle_rad)))) -
       (0.41 * (1.0 - std::exp(-17.0 / config_.aspect_ratio))));

  const double tangent_coeff =
      0.5 * config_.skin_friction_coeff * std::cos(effective_angle_rad);

  const double lift_coeff = normal_coeff * std::cos(effective_angle_rad) -
                            tangent_coeff * std::sin(effective_angle_rad);

  const double drag_coeff = normal_coeff * std::sin(effective_angle_rad) +
                            tangent_coeff * std::cos(effective_angle_rad);

  const double pitch_coeff =
      normal_coeff_to_pitch_coeff_(normal_coeff, effective_angle_rad);

  return DragCoefficients{lift_coeff, drag_coeff, pitch_coeff};
}

double AirfoilElement::flat_plate_drag_coeff_(
    const AirfoilElementState& state) {
  constexpr std::array<double, 3> FLAT_PLATE_TAYLOR_COEFFS = {
      1.98,
      2.1e-1,
      -4.26e-2};
  return FLAT_PLATE_TAYLOR_COEFFS[0] +
         FLAT_PLATE_TAYLOR_COEFFS[1] * state.flap_angle_rad +
         FLAT_PLATE_TAYLOR_COEFFS[2] * state.flap_angle_rad *
             state.flap_angle_rad;
}

double AirfoilElement::normal_coeff_to_pitch_coeff_(
    double normal_coeff,
    double effective_angle_rad) {
  constexpr double FOURTH = 0.25;
  constexpr double EIGHTH = 0.175;
  return normal_coeff *
         (FOURTH - EIGHTH * (1.0 - (TWO * effective_angle_rad / M_PI)));
}

// Fitted to Figure 3.36 in McCormmick (2nd ed.)
double AirfoilElement::viscosity_factor_eta_(
    const AirfoilElementState& state) const {
  constexpr std::array<std::array<double, 4>, FLAP_COUNT>
      AIRFOIL_ETA_TAYLOR_COEFFS = {{
          {0.982857, -1.07202, 0.508055, -0.0522484},
          {0.731429, 0.783036, -2.05565, 0.940448},
          {0.660001, 1.72387, -3.4821, 1.51517},
          {0.608572, -0.364695, 0.17587, 0.05225},
      }};

  const double clamped_flap_angle_rad = std::clamp(
      std::abs(state.flap_angle_rad),
      M_PI / 18.0,
      7.0 * M_PI / 18.0);
  const auto coeff_idx = static_cast<size_t>(config_.flap_type);
  return std::clamp(
      AIRFOIL_ETA_TAYLOR_COEFFS.at(coeff_idx)[0] +
          AIRFOIL_ETA_TAYLOR_COEFFS.at(coeff_idx)[1] * clamped_flap_angle_rad +
          AIRFOIL_ETA_TAYLOR_COEFFS.at(coeff_idx)[2] * clamped_flap_angle_rad *
              clamped_flap_angle_rad +
          AIRFOIL_ETA_TAYLOR_COEFFS.at(coeff_idx)[3] * clamped_flap_angle_rad *
              clamped_flap_angle_rad * clamped_flap_angle_rad,
      0.0,
      1.0);
}

// Fitted to Figure 3.37 in McCormmick (2nd ed.)
double AirfoilElement::d_lift_coeff_max_from_lift_coeff_() const {
  constexpr std::array<std::array<double, 4>, FLAP_COUNT>
      D_LIFT_COEFF_MAX_TAYLOR_COEFFS = {{
          {1.02881, -1.16274, -0.407343, 0.557498},
          {1.01476, 0.00979021, -3.21562, 2.22611},
          {0.995175, 0.614258, -4.39627, 2.81663},
          {1.02881, -1.16274, -0.407343, 0.557498},
      }};

  const double flap_ratio = config_.flap_chord_m / config_.total_chord_m;
  const auto coeff_idx = static_cast<size_t>(config_.flap_type);
  return std::clamp(
      D_LIFT_COEFF_MAX_TAYLOR_COEFFS.at(coeff_idx)[0] +
          D_LIFT_COEFF_MAX_TAYLOR_COEFFS.at(coeff_idx)[1] * flap_ratio +
          D_LIFT_COEFF_MAX_TAYLOR_COEFFS.at(coeff_idx)[2] * flap_ratio *
              flap_ratio +
          D_LIFT_COEFF_MAX_TAYLOR_COEFFS.at(coeff_idx)[3] * flap_ratio *
              flap_ratio * flap_ratio,
      0.0,
      1.0);
}

}  // namespace resim::dynamics::aerodynamics
