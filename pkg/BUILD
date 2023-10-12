# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

load("@rules_python//python:packaging.bzl", "py_package", "py_wheel")
load("//:version.bzl", "version")

py_package(
    name = "metrics_schema_package",
    packages = ["resim.metrics.proto"],
    deps = [
        "//resim/metrics/proto:metrics_proto_py",
        "//resim/metrics/proto:validate_metrics_proto",
    ],
)

py_wheel(
    name = "metrics_schema_wheel",
    distribution = "resim_metrics_schema",
    python_tag = "py3",
    version = version,
    deps = [":metrics_schema_package"],
)

py_package(
    name = "metrics_package",
    packages = ["resim"],
    deps = [
        "//resim/metrics:dice_coefficient",
    ],
)

py_wheel(
    name = "metrics_wheel",
    distribution = "resim_metrics",
    python_tag = "py3",
    version = version,
    deps = [":metrics_package"],
)
