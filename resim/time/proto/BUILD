load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

cc_library(
    name = "time_to_proto",
    srcs = ["time_to_proto.cc"],
    hdrs = ["time_to_proto.hh"],
    visibility = [
        "//visibility:public",
    ],
    deps = [
        "//resim/assert",
        "//resim/time:timestamp",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_test(
    name = "time_to_proto_test",
    srcs = ["time_to_proto_test.cc"],
    deps = [
        ":time_to_proto",
        "//resim/time:random_duration",
        "@com_google_googletest//:gtest_main",
    ],
)
