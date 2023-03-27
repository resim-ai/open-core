load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

cc_library(
    name = "step_executor",
    hdrs = ["step_executor.hh"],
    visibility = ["//resim_core/simulator:__subpackages__"],
    deps = [
    ],
)

cc_library(
    name = "simple_step_executor",
    srcs = ["simple_step_executor.cc"],
    hdrs = ["simple_step_executor.hh"],
    visibility = ["//resim_core/simulator:__subpackages__"],
    deps = [
        ":step_executor",
    ],
)

cc_test(
    name = "simple_step_executor_test",
    srcs = ["simple_step_executor_test.cc"],
    deps = [
        ":simple_step_executor",
        "//resim_core/utils:inout",
        "//resim_core/utils:uuid",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "channel",
    hdrs = ["channel.hh"],
    visibility = ["//resim_core/simulator:__subpackages__"],
    deps = [
    ],
)

cc_test(
    name = "channel_test",
    srcs = ["channel_test.cc"],
    deps = [
        ":channel",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "channel_registry",
    srcs = ["channel_registry.cc"],
    hdrs = ["channel_registry.hh"],
    visibility = ["//resim_core/simulator:__subpackages__"],
    deps = [
        ":channel",
        "//resim_core/assert",
        "@com_github_google_glog//:glog",
    ],
)

cc_test(
    name = "channel_registry_test",
    srcs = ["channel_registry_test.cc"],
    deps = [
        ":channel",
        ":channel_registry",
        "//resim_core/assert",
        "@com_google_googletest//:gtest_main",
    ],
)
