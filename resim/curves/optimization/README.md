# Curve Optimization

The aim of this folder is to support the optimization of a TCurve to match a sequence of timed poses.
To do this, we make use of [Gauss-Newton Optimization](https://ethaneade.com/optimization.pdf) as
implemented [here](/resim/math/gauss_newton_optimizer.hh). To do this, we need two main components:

 - A parameter implementation located in [control_point_parameter.hh](control_point_parameter.hh).
   This gives us a type that behaves like a `TCurve<SE3>::Control` but which has the interface
   expected by the `GaussNewtonOptimizer` to create a `ParameterBlock<ControlPointParameter>` so we
   can find the TCurve that minimizes our objective.
 - An error model implementation located in [pose_error_model.hh](pose_error_model.hh).
   This defines our objective function. It is constructed with a series of timed
   pose observations that the TCurve is expected to follow. This relies on the
   [t_curve_differential.hh](t_curve_differential.hh) library to do the heavy lifting when it comes
   to computing the TCurve's pose at the time of each pose observation and the derivatives of this
   pose with respect to the control point parameters. Specifically, this error model insists that
   the $\text{SE}(3)$ logarithm of `observed_frame_from_scene * t_curve_from_scene.inverse()` be as
   small as possible.

Once we have these, we can register them with the `GaussNewtonOptimizer` and fit curves to our
heart's content. To get an idea of what this looks like, you can run:

```
bazel build //resim/curves/optimization:pose_error_model_test
./bazel-bin/resim/curves/optimization/pose_error_model_test
```

This will create a file `vis.mcap` in your current working directory which can be opened in
[Foxglove Studio](https://app.foxglove.dev/) to see the results of such a TCurve fitting.
