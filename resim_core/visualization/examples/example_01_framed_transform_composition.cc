
#include <cstdlib>

#include "resim_core/transforms/framed_group.hh"
#include "resim_core/transforms/se3.hh"
#include "resim_core/transforms/so3.hh"
#include "resim_core/visualization/view.hh"
int main(int argc, char *argv[]) {
  // Now let's try composing some transforms. Suppose we have a robot with a
  // sensor on it. And we want to know where they are in the world. We can make
  // this easier by defining frames for the things we care about:
  using Frame = resim::transforms::Frame<3>;  // ReSim's frame identifier.
  const Frame world = Frame::new_frame();
  const Frame robot = Frame::new_frame();
  const Frame sensor = Frame::new_frame();

  // ReSim VIEW enables you to name the frames, for more convenient
  // representation in foxglove. The underlying resim::transforms::Frame<DIMS>
  // C++ type does not store a name on its own to keep the datatype as
  // lightweight as possible. This means that anywhere the frame is used in a
  // transform the meaning will be attached.
  VIEW(world) << "world";
  VIEW(robot) << "robot";
  VIEW(sensor) << "sensor";

  //  ReSim has special versions of its SE3 and SO3 transforms that are
  //  explicitly framed:
  using resim::transforms::FSE3;  // ReSim's framed 6 d.o.f. rigid xform.
  using resim::transforms::SE3;   // ReSim's 6 d.o.f. rigid xform..
  using resim::transforms::SO3;   // ReSim's 3 d.o.f. rotation.

  // Framed transforms have two big advantages as we will see in the example
  // below.....

  // Create a simple rotation of pi/2 about the Z-axis.
  const double ANGLE_RAD = M_PI_2;
  const double AXIS_X = 0.0;
  const double AXIS_Y = 0.0;
  const double AXIS_Z = 1.0;

  const SO3 world_from_robot_rot(ANGLE_RAD, {AXIS_X, AXIS_Y, AXIS_Z});

  // Combine the rotation with a simple translation {x, y, z} to construct an
  // SE3 and also specify the frames to build a framed SE3 (FSE3).
  const double TRANSLATION_X = 1.0;
  const double TRANSLATION_Y = 0.0;
  const double TRANSLATION_Z = 1.0;
  const FSE3 world_from_robot(
      SE3(world_from_robot_rot, {TRANSLATION_X, TRANSLATION_Y, TRANSLATION_Z}),
      world,
      robot);

  // Visualize the resultant transform.
  //
  // The name provided for a framed transform is used for the topic list in
  // foxglove, whereas the names of the frames are displayed in the 3D pane.
  VIEW(world_from_robot) << "world_from_robot";

  // Next Let's do a similar thing for the sensor:
  const double SENSOR_X = 0.1;
  const double SENSOR_Y = 0.0;
  const double SENSOR_Z = 0.1;
  const FSE3 robot_from_sensor(
      SE3({SENSOR_X, SENSOR_Y, SENSOR_Z}),
      robot,
      sensor);

  // Note that there is no rotation between the robot and the sensor so we
  // simply omit it from the constructor.

  // Now let's try visualizing the new transform:
  VIEW(robot_from_sensor) << "robot_from_sensor";

  // This is where we see ADVANTAGE #1 of explicit framing. Notice how the
  // visualization displays the relative poses of the robot and sensor correctly
  // the frame specification allows ReSim to build a scene graph on-the-fly.

  // Now let's try a composition operation
  FSE3 world_from_sensor = world_from_robot * robot_from_sensor;

  // Visualize this to sanity check that the composed transform matches the
  // other two....
  VIEW(world_from_sensor) << "world_from_sensor";

  // This is where we see ADVANTAGE #2 of explicit framing. Although you did not
  // see anything, ReSim used the frames to check that the composition operation
  // was valid. If users accidentally specify an invalid composition (the source
  // of many frustrating transforms bugs) ReSim will throw an error. Uncomment
  // the code below to see and example of this.

  /*
  // Danger! The code below contains a deliberate bug.
  const FSE3 sensor_from_robot = robot_from_sensor.inverse();
  // !!Invalid composition!!
  const FSE3 bad_world_from_sensor =
      world_from_robot * sensor_from_robot;
  */
  return EXIT_SUCCESS;
}
