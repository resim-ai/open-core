load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "timestamp",
    hdrs = ["timestamp.hh"],
    visibility = ["//visibility:public"],
    deps = [],
)

cc_library(
    name = "event_schedule",
    hdrs = ["event_schedule.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":timestamp",
    ],
)

cc_test(
    name = "event_schedule_test",
    srcs = ["event_schedule_test.cc"],
    deps = [
        ":event_schedule",
        ":timestamp",
        "@com_google_googletest//:gtest_main",
    ],
)
