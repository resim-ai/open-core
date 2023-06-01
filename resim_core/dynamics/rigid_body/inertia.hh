////////////////////////////////////////////////////////////////////////////////
// inertia.hh                                                                 //
////////////////////////////////////////////////////////////////////////////////
//
// This file defines an alias and utilities for working with 6 by 6 inertia
// matrices used for 3D rigid body mechanics. In particular, these matrices are
// defined such that, in a center-of-mass reference frame (i.e. a frame whose
// origin is at a body's center of mass, and whose axes are the principal axes
// of rotation), they have the form:
//
//               [I_x    0       0       0       0       0]
//               [0      I_y     0       0       0       0]
//               [0      0       I_z     0       0       0]
//               [0      0       0       M       0       0]
//               [0      0       0       0       M       0]
//               [0      0       0       0       0       M]
//
// where I_x, I_y, and I_z are the x, y, and z moments of inertia, and M is the
// total mass. Note that inertia matrices can generally be non-diagonal (if the
// coordinate frame they're expressed in is not a center-of-mass
// frame). However, they will always be Positive Definite and satisfy some other
// constraints as well.
//
#pragma once

#include <Eigen/Dense>

#include "resim_core/transforms/se3.hh"

namespace resim::dynamics::rigid_body {

using Inertia =
    Eigen::Matrix<double, transforms::SE3::DOF, transforms::SE3::DOF>;

// Convert a mass and moments of inertia into an Inertia matrix. Essentially,
// this constructs the matrix shown above.
// @param[in] mass - The mass of the body.
// @param[in] moments_of_inertia - The principal moments of inertia of the body.
Inertia inertia_from_mass_and_moments_of_inertia(
    double mass,
    const Eigen::Vector3d &moments_of_inertia);

}  // namespace resim::dynamics::rigid_body
