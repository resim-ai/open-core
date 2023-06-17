// This file is a companion for the View documentation at
// https://docs.resim.ai/visualization/

#include "resim/visualization/view.hh"

#include "resim/transforms/frame.hh"
#include "resim/transforms/se3.hh"
#include "resim/transforms/so3.hh"

using resim::transforms::SE3;
using resim::transforms::SO3;
using Frame = resim::transforms::Frame<SE3::DIMS /* 3 */>;

int main(int argc, char **argv) {
  //////////////////////////////////////////////////////////////////////////////
  // Viewing Transforms
  //////////////////////////////////////////////////////////////////////////////
  {
    // Generate new frames with unique ids:
    const Frame scene = Frame::new_frame();
    const Frame robot = Frame::new_frame();

    const SE3 scene_from_robot{
        SO3::identity(),  // Rotation
        {1., 0., 1.},     // Translation
        scene,            // Destination/Into Frame
        robot             // Source/From Frame
    };

    // We're using the << operator to add a name to our frame. If we don't use
    // it, the frame id is used instead.
    VIEW(scene) << "Scene Frame";
    VIEW(robot);
    VIEW(scene_from_robot) << "My Transform";
  }

  //////////////////////////////////////////////////////////////////////////////
  // Building a Scene Graph
  //////////////////////////////////////////////////////////////////////////////
  {
    // Generate new frames with unique ids:
    const Frame scene = Frame::new_frame();
    const Frame robot = Frame::new_frame();
    const Frame sensor = Frame::new_frame();

    const SE3 scene_from_robot{
        SO3::identity(),  // Rotation
        {1., 0., 1.},     // Translation
        scene,            // Destination/Into Frame
        robot             // Source/From Frame
    };

    const SE3 robot_from_sensor{
        SO3(M_PI, {0., 0., 1}),  // Rotation
        {-0.25, 0.25, 0.25},     // Translation
        robot,                   // Destination/Into Frame
        sensor                   // Source/From Frame
    };

    // We're using the << operator to add a name to our frame. If we don't use
    // it, the id is used instead.
    VIEW(scene) << "Scene Frame";
    VIEW(robot) << "Robot";
    VIEW(scene_from_robot) << "Scene from Robot";

    VIEW(sensor) << "Sensor";
    VIEW(robot_from_sensor) << "Robot from Sensor";
  }

  return EXIT_SUCCESS;
}
