# Copyright 2026 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

load("@resim_python_client_deps//:requirements.bzl", client_requirement = "requirement")
load("@resim_python_deps//:requirements.bzl", "requirement")
load("@rules_python//python:defs.bzl", "py_binary", "py_library", "py_test")

py_library(
    name = "demo",
    srcs = [
        "__init__.py",
        "__main__.py",
        "bundle.py",
        "links.py",
        "run.py",
    ],
    data = glob(["data/**"]),
    visibility = ["//visibility:public"],
    deps = [
        "//resim/sdk",
        "//resim/sdk:client",
        "//resim/sdk/auth:device_code_client",
        "//resim/sdk/bff_client:dashboards",
        "//resim/sdk/metrics:emissions",
        client_requirement("httpx"),
    ],
)

py_test(
    name = "demo_test",
    size = "small",
    srcs = [
        "bundle_test.py",
        "config_test.py",
        "demo_test.py",
        "links_test.py",
        "main_test.py",
        "run_test.py",
    ],
    deps = [
        ":demo",
        client_requirement("httpx"),
        requirement("pyyaml"),
    ],
)

# Development tools for regenerating the demo's data bundle. Deliberately not a
# dependency of :demo, so they stay out of the published wheels.
py_binary(
    name = "export_source_batches",
    srcs = [
        "tools/__init__.py",
        "tools/export_source_batches.py",
    ],
    main = "tools/export_source_batches.py",
    deps = [
        "//resim/sdk:client",
        "//resim/sdk/auth:device_code_client",
        "//resim/sdk/bff_client:graphql",
        client_requirement("httpx"),
    ],
)

py_binary(
    name = "build_bundle",
    srcs = [
        "tools/__init__.py",
        "tools/build_bundle.py",
    ],
    main = "tools/build_bundle.py",
    deps = [
        "//resim/sdk/metrics:emissions",
        requirement("pyyaml"),
    ],
)
