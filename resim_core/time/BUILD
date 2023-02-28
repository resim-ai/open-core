load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

cc_library(
    name = "timestamp",
    hdrs = ["timestamp.hh"],
    visibility = ["//visibility:public"],
    deps = [],
)

cc_test(
    name = "timestamp_test",
    srcs = ["timestamp_test.cc"],
    deps = [
        ":random_duration",
        ":timestamp",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "random_duration",
    testonly = True,
    hdrs = ["random_duration.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":timestamp",
        "//resim_core/utils:inout",
    ],
)

cc_library(
    name = "event_schedule",
    hdrs = ["event_schedule.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":timestamp",
        "//resim_core/assert",
    ],
)

cc_test(
    name = "event_schedule_test",
    srcs = ["event_schedule_test.cc"],
    deps = [
        ":event_schedule",
        ":timestamp",
        "//resim_core/assert",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "sample_interval",
    srcs = ["sample_interval.cc"],
    hdrs = ["sample_interval.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":timestamp",
        "//resim_core/assert",
        "//resim_core/math:safe_integer_utils",
    ],
)

cc_test(
    name = "sample_interval_test",
    srcs = ["sample_interval_test.cc"],
    deps = [
        ":sample_interval",
        ":timestamp",
        "@com_google_googletest//:gtest_main",
    ],
)
