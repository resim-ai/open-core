# Using SO(3) and SE(3)

## Introduction

This document is intended to breifly outline the basics of using ReSim's main
Lie group libraries. For an introduction to Lie groups and the reasons why we
think they're worth using, the reader is highly encouraged to read [Lie
Groups](./liegroups.md).

As a motivating example, we imagine that we have the pose of a robot in 3D
space given by Tait-Bryan [Euler
Angles](https://en.wikipedia.org/wiki/Euler_angles) (yaw, pitch, and roll)
$\psi$, $\theta$, and $\varphi$ and the actor's location coordinates $x$, $y$,
and $z$ expressed in some stationary scene coordinate frame. We will show how
to create an SO3 describing the orientation of this robot and an SE3 describing
its full pose. 

## Orientation / SO3

Before we get to the full pose, it's simpler to take on the orientation part
separately. Our Euler angles are defined as the composition of three rotations.
If we have a vector $v_{\text{robot}}$ in our robot's coordinates and we want
to express it in our scene coordinates, we have:

$$
v_\text{scene} = R_x(\varphi) R_y(\theta) R_z(\psi) v_\text{robot}
$$

Where $R_x(\varphi)$ is a rotation matrix for a rotation around the $x$ axis by
$\varphi$ radians. Hence the overall rotation matrix for our orientation is given by:

$$
R = R_x(\varphi) R_y(\theta) R_z(\psi) 
$$

One can identify the colums of this matrix as the axes of the robot coordinate
frame expressed in scene coordinates. Since rotation matrices are a
*representation* of $\text{SO(3)}$, we can use this equation to compute an
`SO3` for our robot pose like so:

```
#include <cmath>

#include "resim/transforms/so3.hh"
#include "resim/visualization/view.hh"

using namespace resim::transforms;

// ...

const double psi = M_PI_4;
const double theta = 0.5;
const double phi = 0.1;

const SO3 scene_from_robot_rotation = SO3(phi, {1., 0., 0.}) *
                                      SO3(theta, {0., 1., 0.}) *
                                      SO3(psi, {0., 0., 1.});

// Visualize with ReSim View
VIEW(scene_from_robot_rotation) << "My rotation";
```

Here, we're constructing `SO3`s representing the rotations around each
axis. Note that we have to treat each rotation in order if we're using Euler
angles. If we use the exponential map on `SO3` (which converts angle/axis
representations of an orientation into an `SO3` object) with a vector of the
angles, we will get a different result:

```
const SO3 scene_from_robot_rotation_wrong = SO3::exp({phi, theta, psi});

// The following assertion will not fail except in the degenerate case of two of
// the angles being zero.
REASSERT(
    not scene_from_robot_rotation.is_approx(scene_from_robot_rotation_wrong));
```

The naming here (i.e. `scene_from_robot`) is conventional in ReSim's libraries
as it reflects the fact that the SO3 is a transformation from robot coordinates
into scene coordinates. In fact, one can use the `SO3` to directly transform
vectors.

```
using Vec3 = Eigen::Vector3d;

const Vec3 robot_forward_in_robot_coordinates{1.0, 0.0, 0.0};
Vec3 robot_forward_in_scene_coordinates =
    scene_from_robot_rotation * robot_forward_in_robot_coordinates;

// Can also be more explicit. This is equivalent to the above.
robot_forward_in_scene_coordinates =
    scene_from_robot_rotation.rotate(robot_forward_in_robot_coordinates);
```

Quaternions are also often used to represent orientations, and have many
benefits over Euler angles. Fortunately, `SO3` objects can also be converted
to and from quaternions:

```
const Eigen::Quaterniond scene_from_robot_quat{
    scene_from_robot_rotation.quaternion()};
REASSERT(SO3(scene_from_robot_quat).is_approx(scene_from_robot_rotation));
```

Furthermore, like quaternions, SO3 affords smooth interpolation without
wrap-around issues. To do this, we use the `SO3::interp()` member function:

```
constexpr double EPSILON = 1e-2;
const Vec3 axis = Vec3(0.1, 0.2, 0.3).normalized();
const SO3 scene_from_a(M_PI - EPSILON, axis);
const SO3 scene_from_b(-M_PI + EPSILON, axis);

constexpr double FRACTION = 0.5;
const SO3 scene_from_interped_rotation{
    scene_from_a *
    (scene_from_a.inverse() * scene_from_b).interp(FRACTION)};

const SO3 expected(M_PI, axis);
REASSERT(scene_from_interped_rotation.is_approx(expected));
```

## Pose / SE3

Now, if we want to express the robot's whole pose, most of the work is already
done:
```
#include "resim/transforms/se3.hh"
// ...
const Vec3 scene_from_robot_translation{x, y, z};
const SE3 scene_from_robot{
    scene_from_robot_rotation,
    scene_from_robot_translation};
```

This can still be used to transform points as above, but as one might expect,
the translation is now included:

```
const Vec3 point_in_robot_coordinates{Vec3::Random()};
const Vec3 point_in_scene_coordinates{
    scene_from_robot * point_in_robot_coordinates};
REASSERT(
    point_in_scene_coordinates ==
    scene_from_robot_rotation * point_in_robot_coordinates +
        scene_from_robot_translation);
```

Sometimes, we don't want to transform a point, but instead we want to transform
a vector (e.g. robot velocity). In this case, we instead use:

```
const Vec3 vector_in_robot_coordinates{Vec3::Random()};
const Vec3 vector_in_scene_coordinates{
    scene_from_robot.rotate(vector_in_robot_coordinates)};
REASSERT(
    vector_in_scene_coordinates ==
    scene_from_robot_rotation * vector_in_robot_coordinates);
```

Note that the `rotate()` member function is different than `operator*()` for
`SE3` even though it is equivalent for `SO3`.

Like `SO3`, `SE3` also has an `interp()` member function which can be used to
interpolate elements of the group:

```
const SE3 scene_from_interped{scene_from_robot.interp(0.5)};
REASSERT(
    scene_from_robot.is_approx(scene_from_interped * scene_from_interped));
```

Note that interpolation of the Lie group $\text{SE(3)}$ follows a smooth
geodesic curve between the two frames. This property can confer some advantages
 - especially in robotics applications - but, it's important the caller is aware
of this.

## Derivatives

So far, we have dealt with the static pose of an example robot relative to the
scene. A natural next step is to discuss how to work with time derivatives of
poses since robots typically have components that change their poses over time.
We very frequently use objects called *tangent vectors* to express this
velocity information. We'll define these vectors here without diving deeply
into the math (for more see [Lie Group
Derivatives](./liegroup_derivatives.md)). For $\text{SE(3)}$, we can represent
any element/object $g$ with a 4x4 matrix of the form:

$$
g =
\begin{bmatrix}
R & t \\
0 & 1 \\
\end{bmatrix}
$$

Where $R$ is a 3x3 [rotation
matrix](https://en.wikipedia.org/wiki/Rotation_matrix), and $t$ is the 3x1
translation vector. To operate on a point $p$, we simply append 1 to it and we
can see:

$$
g * p=
\begin{bmatrix}
R & t \\
0 & 1 \\
\end{bmatrix}
\begin{bmatrix}
p \\ 1 \\
\end{bmatrix} = 
\begin{bmatrix}
Rp + t \\ 1
\end{bmatrix}
$$

Because we can represent our poses using a matrix, we now have a way to
represent their derivatives. We can just store the ordinary time derivatives of
the matrix. This will involve us storing 12 floating point numbers: 9 for the
rotation matrix and 3 for the translation. We can do better however. After all,
we know that the rotation time derivatives (a.k.a. the angular velocity) should
be expressible with only three numbers. There's a trick that we can use here.
If we multiply our ordinary time derivative $\dot g$ by $g^{-1}$, we get a
simpler form:

$$
g^{-1} \dot g = 
\begin{bmatrix}
 0& -\omega_3 & \omega_2 & v_1\\
 \omega_3 & 0& -\omega_1 & v_2\\
 -\omega_2 & \omega_1 & 0& v_3\\
 0 & 0 & 0 & 0& \\
\end{bmatrix}
$$

Now we only have to store six number in what we call a **right** tangent
vector:

$$
\begin{bmatrix}
\omega_1 & \omega_2 & \omega_3 & v_1 & v_2 & v_3
\end{bmatrix} ^T
$$

This is a "right" tangent vector because the ordinary time derivative $\dot g$
appears on the right side of the expression $g^{-1} \dot g$. As you might
imagine, there are also "left" tangent vectors because the expression $\dot g
g^{-1}$ happens to also produce matrices of exactly the above form. However,
the left and right tangent vector representations of the time derivative are in
general different for a given robot trajectory, so care must be taken when
dealing with them. For simplicity, we only deal with right tangent vectors
here.

One can verify that if $g$ as written above is `scene_from_robot`, then
$\omega_1$, $\omega_2$, and $\omega_3$ in the right tangent vector represent
the components of the robot's angular velocity expressed in its own
coordinates. If, for example, they are $[1, 0, 0]$, then the robot is rolling
to its right (assuming the x axis is forward). One can further verify that
$v_1$, $v_2$, and $v_3$ are the robot's linear velocity expressed in its own
coordinates. If they are $[1, 0, 0]$ the actor is moving along its own x axis.

When working with these tangent vectors in code, we often want to easily access
the components from them. For example, let's say we want to know what the
velocity $v$ and angular velocity $\omega$ are in **scene** coordinates. The
paragraph above tells us how to get these velocities in robot coordinates, and
then we need to rotate them to get them into scene coordinates. The variable
`d_scene_from_robot` holds a right tangent vector representing our time
derivative (e.g. $g^{-1}\dot g$ if $g$ represents `scene_from_robot`). `SE3` is
equipped with helpers to get these components from a tangent vector:

```
const SE3::TangentVector d_scene_from_robot = SE3::TangentVector::Random();

const Vec3 robot_angular_velocity_in_robot_coordinates{
    SE3::tangent_vector_rotation_part(d_scene_from_robot)};
const Vec3 robot_velocity_in_robot_coordinates{
    SE3::tangent_vector_translation_part(d_scene_from_robot)};

const Vec3 robot_angular_velocity_in_scene_coordinates{
    scene_from_robot.rotation() *
    robot_angular_velocity_in_robot_coordinates};

const Vec3 robot_velocity_in_scene_coordinates{
    scene_from_robot.rotation() * robot_velocity_in_robot_coordinates};
```

If we want to go in the opposite direcion, we can also do that:

```
REASSERT(
    d_scene_from_robot == SE3::tangent_vector_from_parts(
                              robot_angular_velocity_in_robot_coordinates,
                              robot_velocity_in_robot_coordinates));
```

!!! Note
    For $\text{SO(3)}$, the tangent vector matrices have a simpler form:

    $$
    g^{-1} \dot g = \begin{bmatrix}
     & -\omega_3 &  \omega_2  \\
     \omega_3 & & -\omega_1 \\
     -\omega_2 & \omega_1 & \\
     \end{bmatrix}
    $$

    Where $\omega$ is once again the angular velocity as described above.

## Exp and Log

We touched breifly above on the fact that the exponential on $\text{SO(3)}$
converts angle axis representations to elements of $\text{SO(3)}$, but we did
not fully explain this exponential operation that the `SE3` and `SO3` classes
both implement. This operation takes a tangent vector (i.e. a time derivative)
and tells you where you end up in the group following that constant time
derivative for one unit of time. More explicitly, if we think about the idea of
the the matrix representation of the right tangent vector $g^{-1} \dot g$ (as
described above) being constant along a trajectory, we realize that this
statement is a matrix ordinary differential equation
(ODE):

$$
g^{-1} \dot g = \text{Constant} = X
$$

$$
\dot g  = g X
$$

This matrix ODE has a known solution given by the [Matrix
exponential](https://en.wikipedia.org/wiki/Matrix_exponential):

$$
g = g_0\text{Exp}(Xt) = g_0\left( \sum_{k = 0}^{\infty} \frac{1}{k!} X^k t^k\right)
$$

Where $g_0$ is some initial condition for $g$. The exponential map $\exp()$
defined for `SE3` and `SO3` is the same as this matrix exponential up to the
input formatting. For instance, for $\text{SO(3)}$:

$$
\exp\left(\begin{bmatrix}
\omega_1 \\ \omega_2 \\ \omega_3
\end{bmatrix}\right)
= \text{Exp}\left(\begin{bmatrix}
     & -\omega_3 &  \omega_2  \\
     \omega_3 & & -\omega_1 \\
     -\omega_2 & \omega_1 & \\
     \end{bmatrix}\right)
$$

In practice, we don't have to compute an infinite series for $\exp()$ because
there are closed forms for $\text{SO(3)}$ (the [Rodrigues
formula](https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula)) and
$\text{SE(3)}$.

We also provide the logarithm function, which is the inverse of $\exp()$. In
other words, it gives the constant velocity needed to arrive at the given
element from the identity in one time unit. For their complex definitions,
using these in code is quite simple:

```
  REASSERT(my_tangent_vector.isApprox(SE3::exp(my_tangent_vector).log()));
```

## Frame Checking

So far we've ignored the fact that our `SO3` and `SE3` classes come equipped with
frame checking capabilities (as alluded to in [Lie Groups](./liegroups.md)). In
brief, each `SO3` and `SE3` object can be assigned two coordinate frames,
represented by `Frame` objects. This is done by passing the coordinate frames
into the constructor or into the exponential or identity member functions. When
assigned, these objects ensure that frame consistency is maintained when
composing Lie groups. Here's an example:

```
#include "resim/assert/assert.hh"
#include "resim/transforms/frame.hh"
#include "resim/transforms/se3.hh"
#include "resim/transforms/so3.hh"
#include "resim/visualization/view.hh"

using resim::transforms::SE3;
using resim::transforms::SO3;
using Frame = resim::transforms::Frame<SE3::DIMS>; /* SE3::DIMS == 3 */

// ...

const Frame world{Frame::new_frame()};
const Frame robot{Frame::new_frame()};
const Frame sensor{Frame::new_frame()};

// The pose of the robot in the world
const SE3 world_from_robot{SO3::identity(), {5., 5., 0.}, world, robot};

// Visualize with ReSim View
VIEW(world) << "World frame";
VIEW(robot) << "Robot frame";
VIEW(world_from_robot) << "World from robot";

// The pose of a sensor mounted on the robot
const SE3 robot_from_sensor{
    SO3{M_PI_2, {0., 0., 1.}},
    {0., 0., 1.},
    robot,
    sensor};

// Visualize with ReSim View
VIEW(sensor) << "Sensor frame";
VIEW(robot_from_sensor) << "Robot from sensor";

const SE3 world_from_sensor{world_from_robot * robot_from_sensor};
REASSERT(world_from_sensor.is_framed());
REASSERT(world_from_sensor.into() == world);
REASSERT(world_from_sensor.from() == sensor);

// Whoops! This fails at run time because we forgot to invert
// robot_from_sensor!
// const SE3 robot_from_world{world_from_sensor * robot_from_sensor};

// We should have done:
const SE3 robot_from_world{world_from_sensor * robot_from_sensor.inverse()};
```

Using frame checking is generally good practice as it makes it less likely for
silly bugs to occur. In the future, we plan on making it possible to deactivate
frame checking as an optional performance optimization. Note that composition
with an unframed `SO3` or `SE3` *always* results in an unframed `SO3`/`SE3`.
Consequently, unframed objects can propogate rapidly if one is not deliberate
about using framed objects.

!!! Note
    Feel free to play around with the [source
    code](https://github.com/resim-ai/open-core/blob/main/resim/examples/liegroups.cc)
    for the examples above.
