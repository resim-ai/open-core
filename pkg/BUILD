# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

load("@resim_version//:defs.bzl", version = "RESIM_VERSION")
load("@rules_pkg//pkg:mappings.bzl", "pkg_files")
load("@rules_python//python:packaging.bzl", "py_package", "py_wheel")
load("@rules_python//python/entry_points:py_console_script_binary.bzl", "py_console_script_binary")

# This "7 spaces and a pipe" format is needed to make the resulting metadata to
# be parsable by RFC822 parser when using multiple lines. See
# https://packaging.python.org/en/latest/specifications/core-metadata/#description
LICENSE = """Copyright 2025 ReSim, Inc.
       |
       |Permission is hereby granted, free of charge, to any person obtaining a copy of
       |this software and associated documentation files (the "Software"), to deal in
       |the Software without restriction, including without limitation the rights to
       |use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
       |of the Software, and to permit persons to whom the Software is furnished to do
       |so, subject to the following conditions:
       |
       |The above copyright notice and this permission notice shall be included in all
       |copies or substantial portions of the Software.
       |
       |THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
       |IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
       |FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
       |AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
       |LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
       |OUT OF OR IN CONNECTION WITH THE SOFTWAREOR THE USE OR OTHER DEALINGS IN THE
       |SOFTWARE."""

py_package(
    name = "resim_package",
    packages = ["resim"],
    deps = [
        "//resim/actor/state/proto:observable_state_proto_py",
        "//resim/auth/python:device_code_client",
        "//resim/auth/python:username_password_client",
        "//resim/experiences/proto:experience_proto_py",
        "//resim/geometry/python:polygon_distance_python.so",
        "//resim/metrics:default_report_metrics",
        "//resim/metrics:dice_coefficient",
        "//resim/metrics:fetch_all_pages",
        "//resim/metrics:fetch_job_metrics",
        "//resim/metrics:fetch_metrics_urls",
        "//resim/metrics:fetch_report_metrics",
        "//resim/metrics:fetch_sweep_metrics",
        "//resim/metrics:get_metrics_proto",
        "//resim/metrics/proto:metrics_proto_py",
        "//resim/metrics/proto:validate_metrics_proto",
        "//resim/metrics/python:emissions",
        "//resim/metrics/python:metrics",
        "//resim/metrics/python:metrics_utils",
        "//resim/metrics/python:metrics_writer",
        "//resim/metrics/python:plotly_helpers",
        "//resim/metrics/python:unpack_metrics",
        "//resim/msg:detection_proto_py",
        "//resim/msg:header_proto_py",
        "//resim/msg:navsat_proto_py",
        "//resim/msg:odometry_proto_py",
        "//resim/msg:pose_proto_py",
        "//resim/msg:primitives_proto_py",
        "//resim/msg:transform_proto_py",
        "//resim/orchestration:kickoff_report",
        "//resim/transforms/proto:framed_vector_3_proto_py",
        "//resim/transforms/python:geodetic_python.so",
        "//resim/transforms/python:quaternion.so",
        "//resim/transforms/python:se3_python.so",
        "//resim/transforms/python:so3_python.so",
        "//resim/utils/proto:uuid_proto_py",
    ],
)

pkg_files(
    name = "resim_package_stubs",
    srcs = [
        "//resim/geometry/python:box_box_distance_python.stubs",
        "//resim/geometry/python:oriented_box_python.stubs",
        "//resim/geometry/python:polygon_distance_python.stubs",
        "//resim/transforms/python:geodetic_python.stubs",
        "//resim/transforms/python:quaternion.stubs",
        "//resim/transforms/python:se3_python.stubs",
        "//resim/transforms/python:so3_python.stubs",
    ],
    strip_prefix = ".",
)

genrule(
    name = "combined_requires",
    srcs = [
        "//:requirements.txt",
        "//resim-python-client:requirements.txt",
        ":extra_requires.txt",
    ],
    outs = ["combined_requirements.txt"],
    cmd = "cat $(SRCS) > $@",
)

PY_VERSION_TAG = select({
    "@rules_python//python/config_settings:is_python_3.10": "cp310",
    "@rules_python//python/config_settings:is_python_3.11": "cp311",
    "@rules_python//python/config_settings:is_python_3.12": "cp312",
    "@rules_python//python/config_settings:is_python_3.13": "cp313",
    "//conditions:default": "cp310",
})

py_wheel(
    name = "resim_wheel",
    abi = PY_VERSION_TAG,
    author = "ReSim Developers",
    classifiers = [
        "License :: OSI Approved :: MIT License",
    ],
    distribution = "resim-open-core",
    homepage = "https://www.resim.ai/",
    license = LICENSE,
    platform = select({
        "@platforms//cpu:aarch64": "manylinux2014_aarch64",
        "@platforms//cpu:x86_64": "manylinux2014_x86_64",
    }),
    python_tag = PY_VERSION_TAG,
    requires_file = ":combined_requires",
    strip_path_prefixes = ["resim-python-client/resim-python-client-gen"],
    summary = "ReSim Metrics Libraries.",
    twine = "@resim_pkg_deps//twine",
    version = version,
    deps = [
        ":resim_package",
        "//resim:py_typed_marker",
    ] + select({
        "//:nocross": [":resim_package_stubs"],
        "//conditions:default": [],
    }),
)

py_package(
    name = "metrics_schema_package",
    packages = ["resim"],
    deps = [
        "//resim/metrics/proto:metrics_proto_py",
        "//resim/metrics/proto:validate_metrics_proto",
    ],
)

py_wheel(
    name = "metrics_schema_wheel",
    author = "ReSim Developers",
    classifiers = [
        "License :: OSI Approved :: MIT License",
    ],
    distribution = "resim_metrics_schema",
    homepage = "https://www.resim.ai/",
    license = LICENSE,
    python_tag = "py3",
    requires_file = "//:requirements.txt",
    summary = "ReSim Metrics Schema Bindings",
    version = version,
    deps = [":metrics_schema_package"],
)

py_package(
    name = "metrics_package",
    packages = [
        "resim.geometry",
        "resim.metrics",
        "resim.transforms",
        "resim.utils",
    ],
    deps = [
        "//resim/geometry/python:polygon_distance_python.so",
        "//resim/metrics:dice_coefficient",
        "//resim/metrics:fetch_job_metrics",
        "//resim/metrics/python:metrics",
        "//resim/metrics/python:metrics_writer",
        "//resim/metrics/python:unpack_metrics",
        "//resim/transforms/python:geodetic_python.so",
        "//resim/transforms/python:quaternion.so",
        "//resim/transforms/python:se3_python.so",
        "//resim/transforms/python:so3_python.so",
    ],
)

py_wheel(
    name = "metrics_wheel",
    author = "ReSim Developers",
    classifiers = [
        "License :: OSI Approved :: MIT License",
    ],
    distribution = "resim_metrics",
    homepage = "https://www.resim.ai/",
    license = LICENSE,
    platform = select({
        "@platforms//cpu:aarch64": "manylinux2014_aarch64",
        "@platforms//cpu:x86_64": "manylinux2014_x86_64",
    }),
    python_tag = "py3",
    requires_file = "//:requirements.txt",
    summary = "ReSim Metrics Libraries.",
    version = version,
    deps = [":metrics_package"],
)

py_package(
    name = "msg_package",
    packages = [
        "resim.actor.state.proto",
        "resim.curves.proto",
        "resim.msg",
        "resim.transforms.proto",
    ],
    deps = [
        "//resim/actor/state/proto:observable_state_proto_py",
        "//resim/msg:detection_proto_py",
        "//resim/msg:header_proto_py",
        "//resim/msg:navsat_proto_py",
        "//resim/msg:odometry_proto_py",
        "//resim/msg:pose_proto_py",
        "//resim/msg:primitives_proto_py",
        "//resim/msg:transform_proto_py",
        "//resim/transforms/proto:framed_vector_3_proto_py",
    ],
)

py_wheel(
    name = "msg_wheel",
    author = "ReSim Developers",
    classifiers = [
        "License :: OSI Approved :: MIT License",
    ],
    distribution = "resim_msg",
    homepage = "https://www.resim.ai/",
    license = LICENSE,
    python_tag = "py3",
    requires_file = "//:requirements.txt",
    summary = "ReSim Metrics Libraries.",
    version = version,
    deps = [":msg_package"],
)

py_wheel(
    name = "ros2_wheel",
    author = "ReSim Developers",
    classifiers = [
        "License :: OSI Approved :: MIT License",
    ],
    distribution = "resim_ros2",
    homepage = "https://www.resim.ai/",
    license = LICENSE,
    platform = select({
        "@platforms//cpu:aarch64": "manylinux2014_aarch64",
        "@platforms//cpu:x86_64": "manylinux2014_x86_64",
    }),
    python_tag = "py3",
    requires_file = "//:requirements.txt",
    strip_path_prefixes = ["resim/ros2/ros2_py_package/"],
    summary = "ReSim ROS2 Libraries.",
    version = version,
    deps = [
        "//resim/ros2:ros2_py_package",
    ] + select({
        "//:nocross": ["//resim/ros2:ros2_py_package_stubs"],
        "//conditions:default": [],
    }),
)

py_console_script_binary(
    name = "twine",
    pkg = "@resim_pkg_deps//twine",
)
