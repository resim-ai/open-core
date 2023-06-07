# Lie Groups

## Introduction

!!! note "Disclaimer"
    *This is not written from the perspective of a mathematician or
    physicist, but rather from the perspective of an engineer that has found Lie
    groups to be an indespensible tool for robotics. The knowledge here is
    primarily practical knowledge built up over experience, so it is very likely
    that some of the mathematical details are not precisely correct as
    written. Please don't hesitate to reach out and let us know if you find any
    errors!*

[**Lie groups**](https://en.wikipedia.org/wiki/Lie_group) are mathematical
objects with a rich history of study and application. They are named for the
Norwegian mathematician [Sophus Lie](https://en.wikipedia.org/wiki/Sophus_Lie)
who pioneered the mathematics of continuous transformation groups.

Why should we use Lie groups in robotics? The answer is that robotics is replete
with examples of continuous transform groups. One example is the fundamental
concept of pose. A pose describes a rigid-body transformation from one set of
coordinates to another. Most commonly, a robot's location and orientation
relative to the world is represented by the pose (or rigid transformation)
relating the "world" and "robot" frames.

There are, of course, many ways of representing a robot's pose. Orientation
alone can be represented by Euler angles, quaternions, rotation matrices, etc.
Using Lie groups affords us two primary advantages:

 - First, the set of all orientations and the set of all poses
are both Lie groups. Representing them as such allows us to write generic
algorithms (like Hermite spline construction or Newtonian dynamics) that
operate equally well on both since both are equipped with the same basic Lie
group properties. This would be impossible to do, for example, if we
represented orientations as quaternions (which otherwise have very nice
properties) and poses as tuples of (quaternion, translation vector).

 - Second, every Lie group has a corresponding Lie algebra (a vector space), which
can be used to describe its structure locally (i.e. near any given element in
the group). This means that we can do a lot of things on Lie groups that we
normally know how to do on vector spaces. For instance, we can:

     - Construct Gaussian distributions on the Lie group of rigid body
       transformations (SE(3)) and sample from them. 
    
     - Interpolate between recorded actor poses to produce a trajectory without
       worrying about wrap-around issues that might result if we interpolated
       poses with Euler angles describing their orientations.
    
     - Write loss functions on orientations and use them for optimal control, once
       again without worrying about wrap-around or singularities.
    
     - Take the derivatives of our poses and represent them using elements of the
       corresponding Lie algebra.

We've glossed over everything exceptionally briefly here, so I would encourage
the curious reader to peruse the [external links](#external-links) below.

## LieGroup Interface

In an effort to make Lie-groups-related code more reusable, we require each
implementation of a Lie group object (that is an object representing an element
of a Lie group) to satify a few constraints.

First, the implementation must inherit from the LieGroup template. This base
class is templated on the dimensionality of the input to the transform (e.g. 3
for 3D rotations), and the number of degrees of freedom for this group (e.g. 6 =
{3 rotational} + {3 translational} for rigid body transformations). This base
template also defines some helpful aliases for tangent vectors (i.e. members of
the Lie algebra).

Second, the implementation should satisfy the LieGroupType concept which
enforces a number of the group axioms (e.g. group action, invertability,
identity, etc.). We implement these checks as a concept since different groups
need different signatures so inheritance doesn't allow us to enforce them.

## Lie Groups Implemented

**SO(3)** - The Special Orthogonal Group in 3D. The set of all rotations.

**SE(3)** - The Special Euclidian Group in 3D. The set of all rigid transformations.

## Framed Groups

As discussed above, Lie group elements are frequently used to represent
transformations between coordinate frames. In practice, this can often lead to
mistakes when users of Lie group libraries compose group elements in the wrong
order (e.g. multiplying `robot_from_sensor` times `scene_from_robot`). These
small mistakes are simple enough that they should be possible to catch, at least
at runtime. We therefore assign a unique id to each coordinate frame. Group elements 
thus have two frame ids that they track (`into` and `from`). When two elements
are multiplied, consistency is enforced between them. More precisely, if a pose
that transforms points in frame B's coordinates to frame A's coordinates (call
the transform `A_from_B`) and a similar transform `C_from_D` then we should fail
if the user ever tries to multiply `A_from_B * C_from_D`. However `A_from_B *
B_from_D` is fine. Unframed groups are supported, by setting the frame to a null 
(0) id, in which case they are not checked. Be aware that multiplying an unframed
group by a framed group will always result in an unframed group, so unframed groups
can propagate quickly!

## More Information

For more information on the features our Lie group classes provide for
working with the derivatives of functions involving Lie groups, please refer to [this guide](./liegroup_derivatives.md).

## External Links
For more information, please take a look at the following links:

 - [A micro Lie theory for state estimation in robotics](https://drive.google.com/file/d/1UlI1N63o6abyL03VfbYoXu22CcYTdZ6b/view)
 - [ethaneade.com](https://ethaneade.com/)
 - Jakob Schwichtenberg:
     - [Lie Group Theory - A Completely Naive Introduction](https://jakobschwichtenberg.com/naive-introduction-lie-theory/)
     - [How is a Lie Algebra able to describe a group](https://jakobschwichtenberg.com/lie-algebra-able-describe-group/)
     - [What's so special about the adjoint representation of a Lie group?](https://jakobschwichtenberg.com/adjoint-representation/)
