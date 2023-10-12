# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

load("@rules_python//python:packaging.bzl", "py_package", "py_wheel")
load("//:version.bzl", "version")

LICENSE = """Copyright 2023 ReSim, Inc.

         Permission is hereby granted, free of charge, to any person obtaining a copy of
         this software and associated documentation files (the "Software"), to deal in
         the Software without restriction, including without limitation the rights to
         use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
         of the Software, and to permit persons to whom the Software is furnished to do
         so, subject to the following conditions:
         
         The above copyright notice and this permission notice shall be included in all
         copies or substantial portions of the Software.
         
         THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
         IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
         FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
         AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
         LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
         OUT OF OR IN CONNECTION WITH THE SOFTWAREOR THE USE OR OTHER DEALINGS IN THE
         SOFTWARE.
"""

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
    requires = [
        "protobuf>=4.24.4",
    ],
    summary = "ReSim Metrics Schema Bindings",
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
    author = "ReSim Developers",
    classifiers = [
        "License :: OSI Approved :: MIT License",
    ],
    distribution = "resim_metrics",
    homepage = "https://www.resim.ai/",
    license = LICENSE,
    python_tag = "py3",
    requires = [
        "protobuf>=4.24.4",
    ],
    summary = "ReSim Metrics Libraries. Metrics schema bindings included.",
    version = version,
    deps = [":metrics_package"],
)