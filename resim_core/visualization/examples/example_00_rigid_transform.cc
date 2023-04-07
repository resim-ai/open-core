
#include <cstdlib>

#include "resim_core/transforms/se3.hh"
#include "resim_core/transforms/so3.hh"
#include "resim_core/visualization/view.hh"

int main(int argc, char *argv[]) {
  using resim::transforms::SE3;  // ReSim's 6 d.o.f. rigid xform.
  using resim::transforms::SO3;  // ReSim's 3 d.o.f. rotation.

  // Create a simple rotation of pi/2 about the Z-axis.
  const double ANGLE_RAD = M_PI_2;
  const double AXIS_X = 0.0;
  const double AXIS_Y = 0.0;
  const double AXIS_Z = 1.0;

  const double TRANSLATION_X = 1.0;
  const double TRANSLATION_Y = 0.0;
  const double TRANSLATION_Z = 1.0;

  // Initialize our SO3 with an Angle-Axis rotation of ANGLE_RAD radians
  // about the Z axis {0., 0., 1.}.
  const SO3 world_from_robot_rot(ANGLE_RAD, {AXIS_X, AXIS_Y, AXIS_Z});

  // Visualize the rotation on its own by running this line and viewing
  // at app.resim.ai.
  //
  // Similar to Google Logging, VIEW requires you to stream in a name for the
  // object that you would like to view.
  VIEW(world_from_robot_rot) << "world_from_robot_rot";

  // ReSim VIEW offers an alternative syntax, depending on preference:
  //
  // VIEW(world_from_robot_rot,"world_from_robot_rot_alt");

  // Combine the rotation with a simple translation {x, y, z} to make an SE3
  // rigid transform.
  const SE3 world_from_robot(
      world_from_robot_rot,
      {TRANSLATION_X, TRANSLATION_Y, TRANSLATION_Z});

  // Visualize the resultant transform
  // Run this line and view at app.resim.ai
  VIEW(world_from_robot) << "world_from_robot";

  return EXIT_SUCCESS;
}
