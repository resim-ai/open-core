#pragma once

#include <Eigen/Dense>

#include "resim_core/assert/assert.hh"
#include "resim_core/dynamics/aerodynamics/drag_coefficients.hh"
#include "resim_core/dynamics/aerodynamics/rigid_body_aerodynamics.hh"
#include "resim_core/transforms/frame.hh"
#include "resim_core/transforms/framed_group.hh"
#include "resim_core/transforms/framed_vector.hh"

namespace resim::dynamics::aerodynamics {

enum AirfoilFlapType { PLAIN, SLOTTED, DOUBLE_SLOTTED, SPLIT, FLAP_COUNT };

struct AirfoilElementConfig {
  // Input variables
  const double lift_curve_slope_coeff;
  const double skin_friction_coeff;
  const double zero_lift_angle_rad;
  const double stall_angle_pos_rad;
  const double stall_angle_neg_rad;
  const double total_chord_m;
  const double flap_chord_m;
  const double span_m;
  const AirfoilFlapType flap_type;

  // Computed (but public) variables
  const double area_m_squared;
  const double aspect_ratio;
  const double adjusted_lift_curve_slope_coeff;
  const double theta_f_rad;
  const double flap_effectiveness_tau_coeff;
  const double lift_coeff_max_pos;
  const double lift_coeff_max_neg;

  AirfoilElementConfig() = delete;
  AirfoilElementConfig(
      double lift_curve_slope_coeff,
      double skin_friction_coeff,
      double zero_lift_angle_rad,
      double stall_angle_pos_rad,
      double stall_angle_neg_rad,
      double total_chord_m,
      double flap_chord_m,
      double span_m,
      AirfoilFlapType flap_type = AirfoilFlapType::PLAIN);
};

struct AirfoilElementState : public AerodynamicElementState {
  const double &flap_angle_rad;

  explicit AirfoilElementState(const double &angle_rad)
      : flap_angle_rad(angle_rad){};
};

// Represents an airfoil oriented along the x-axis, with z-axis up.
// No force is generated along y-axis.
//
// The underlying Aerodynamics model is based off W. Khan and M. Nahon,
// "Real-time modeling of agile fixed-wing UAV aerodynamics," 2015.
// See: https://ieeexplore.ieee.org/document/7152411
// (doi: 10.1109/ICUAS.2015.7152411.)
//
// TODO(tknowles): As of right now, we do *not* support the pitching moment.
// This will be supported when we move to AerodynamicElements providing
// FSE3::TangentVectors.
class AirfoilElement
    : public AerodynamicElementImpl<AirfoilElement, AirfoilElementState> {
 public:
  static constexpr double AIR_DENSITY_KG_PER_M_CUBED =
      1.204;  // At 20 deg C, and 101.325 kPA.

  AirfoilElement(transforms::FSE3 com_from_cop, AirfoilElementConfig config);

 protected:
  FramedVector aerodynamics_impl_(
      const FramedVector &cop_local_wind,
      const AirfoilElementState &state) const;

  friend class AerodynamicElementImpl<AirfoilElement, AirfoilElementState>;

 private:
  // Config
  AirfoilElementConfig config_;

  // Drag related methods
  DragCoefficients compute_low_angle_drag_coeffs_(
      double angle_of_attack_rad,
      double adjusted_zero_lift_angle_rad) const;

  DragCoefficients compute_beyond_stall_drag_coeffs_(
      const AirfoilElementState &state,
      double angle_of_attack_rad,
      double adjusted_zero_lift_angle_rad,
      double adjusted_stall_angle_pos_rad,
      double adjusted_stall_angle_neg_rad) const;

  // Helper methods for computing drag coefficients
  static double flat_plate_drag_coeff_(const AirfoilElementState &state);

  static double normal_coeff_to_pitch_coeff_(
      double normal_coeff,
      double effective_angle_rad);

  double viscosity_factor_eta_(const AirfoilElementState &state) const;

  double d_lift_coeff_max_from_lift_coeff_() const;
};

}  // namespace resim::dynamics::aerodynamics
