load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "fuzz_helpers",
    testonly = True,
    srcs = ["fuzz_helpers.cc"],
    hdrs = [
        "fuzz_helpers.hh",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":parser",
        ":tags",
        "//resim/math:is_approx",
        "//resim/testing:random_matrix",
        "//resim/time:timestamp",
        "//resim/utils:inout",
        "//resim/utils:match",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_library(
    name = "fuzz_helpers_test_template",
    testonly = True,
    hdrs = ["fuzz_helpers_test_template.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":fuzz_helpers",
        "//resim/utils:inout",
        "@com_google_googletest//:gtest",
    ],
)

cc_library(
    name = "tags",
    hdrs = ["tags.hh"],
    deps = [
    ],
)

cc_library(
    name = "parser",
    hdrs = ["parser.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":tags",
    ],
)
