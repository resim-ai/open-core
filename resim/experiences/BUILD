# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

cc_library(
    name = "actor",
    hdrs = ["actor.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim/utils:uuid",
    ],
)

cc_test(
    name = "actor_test",
    srcs = ["actor_test.cc"],
    deps = [
        ":actor",
        "//resim/utils:uuid",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "ilqr_drone",
    hdrs = ["ilqr_drone.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "@libeigen//:eigen",
    ],
)

cc_library(
    name = "location_condition",
    hdrs = ["location_condition.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim/transforms:se3",
        "//resim/utils:uuid",
    ],
)

cc_library(
    name = "completion_criteria",
    hdrs = ["completion_criteria.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim/experiences:location_condition",
        "//resim/time:timestamp",
    ],
)

cc_library(
    name = "storyboard",
    hdrs = ["storyboard.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":ilqr_drone",
        "//resim/actor/state:trajectory",
        "//resim/utils:uuid",
    ],
)

cc_library(
    name = "dynamic_behavior",
    hdrs = ["dynamic_behavior.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":actor",
        ":completion_criteria",
        ":storyboard",
    ],
)

cc_library(
    name = "experience",
    hdrs = ["experience.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":dynamic_behavior",
        ":geometry",
        "//resim/utils:uuid",
    ],
)

cc_library(
    name = "geometry",
    hdrs = ["geometry.hh"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim/geometry:wireframe",
        "//resim/utils:uuid",
    ],
)
