
# Notes on the patches herein

## `testonly.patch`

This patch exists since, without it, the impl target ends up not being `testonly` if the macro is
passed it. This is due to the way that split_kwargs() gives the testonly argument to the launcher
rather than the target. If the target itself has a testonly dependency it will consequently fail to
build.


## `rmw_implementation_patch.patch`

`rules_ros2`
[patches](https://github.com/mvukov/rules_ros2/blob/6130e8f61c1343fab7809e933c9faf47ab1e9fd4/repositories/patches/rmw_implementation_library_path.patch)
[functions.cpp](https://github.com/ros2/rmw_implementation/blob/4dd5d571a5bfa1a67183acf271dfa442932c7572/rmw_implementation/src/functions.cpp)
in `ros2/rmw_implementation` to only accept a compile-time local define called `RMW_LIBRARY_PATH`
that they define
[here](https://github.com/mvukov/rules_ros2/blob/6130e8f61c1343fab7809e933c9faf47ab1e9fd4/repositories/rmw_implementation.BUILD.bazel#L19)
using bazel. We still want to fallback to this same path by default (i.e. when running in bazel),
but we *need* to still be able to side-load a `RMW_IMPLEMENTATION` path when running pybound
ros2-dependent c++ from our ros2 python wheel. To do this we patch the existing rules_ros2 patch so
that it still checks the `RMW_IMPLEMENTATION` environment variable before falling back to the rmw
implementation in bazel. In [resim/ros2/BUILD](./BUILD), we add an [__init__.py](./__init__.py)
which sets this environment variable whenever anyone uses the ros2 modules of the python package.
