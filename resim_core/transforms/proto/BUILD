load("@rules_cc//cc:defs.bzl", "cc_library", "cc_proto_library", "cc_test")
load("@rules_proto//proto:defs.bzl", "proto_library")

cc_library(
    name = "liegroup_to_proto",
    srcs = ["liegroup_to_proto.cc"],
    hdrs = ["liegroup_to_proto.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":se3_proto_cc",
        ":so3_proto_cc",
        "//resim_core/assert",
        "//resim_core/math/proto:matrix_to_proto",
        "//resim_core/transforms:framed_group",
        "//resim_core/transforms:liegroup_concepts",
        "//resim_core/transforms:se3",
        "//resim_core/transforms:so3",
        "//resim_core/utils:inout",
    ],
)

cc_test(
    name = "liegroup_to_proto_test",
    srcs = ["liegroup_to_proto_test.cc"],
    deps = [
        ":se3_proto_cc",
        ":se3_to_proto",
        ":so3_proto_cc",
        ":so3_to_proto",
        "//resim_core/assert",
        "//resim_core/transforms:liegroup_test_helpers",
        "//resim_core/transforms:se3",
        "//resim_core/transforms:so3",
        "@com_google_googletest//:gtest_main",
    ],
)

proto_library(
    name = "frame_3_proto",
    srcs = ["frame_3.proto"],
    visibility = ["//visibility:public"],
    deps = [
        "//resim_core/utils/proto:uuid_proto",
    ],
)

cc_proto_library(
    name = "frame_3_proto_cc",
    visibility = ["//visibility:public"],
    deps = [":frame_3_proto"],
)

cc_library(
    name = "frame_3_to_proto",
    srcs = ["frame_3_to_proto.cc"],
    hdrs = ["frame_3_to_proto.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":frame_3_proto_cc",
        "//resim_core/assert",
        "//resim_core/transforms:frame",
        "//resim_core/utils/proto:uuid_to_proto",
    ],
)

cc_test(
    name = "frame_3_to_proto_test",
    srcs = ["frame_3_to_proto_test.cc"],
    deps = [
        ":frame_3_proto_cc",
        ":frame_3_to_proto",
        "//resim_core/assert",
        "//resim_core/transforms:frame",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "framed_group_to_proto",
    srcs = ["framed_group_to_proto.cc"],
    hdrs = ["framed_group_to_proto.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":frame_3_to_proto",
        ":fse3_proto_cc",
        ":fso3_proto_cc",
        ":liegroup_to_proto",
        "//resim_core/assert",
        "//resim_core/transforms:framed_group",
        "//resim_core/transforms:framed_group_concept",
        "//resim_core/utils:inout",
    ],
)

cc_test(
    name = "framed_group_to_proto_test",
    srcs = ["framed_group_to_proto_test.cc"],
    deps = [
        ":framed_group_to_proto",
        ":fse3_proto_cc",
        ":fse3_to_proto",
        ":fso3_proto_cc",
        ":fso3_to_proto",
        "//resim_core/assert",
        "//resim_core/transforms:frame",
        "//resim_core/transforms:framed_group",
        "//resim_core/transforms:liegroup_test_helpers",
        "@com_google_googletest//:gtest_main",
    ],
)

proto_library(
    name = "fse3_proto",
    srcs = ["fse3.proto"],
    visibility = ["//visibility:public"],
    deps = [
        ":frame_3_proto",
        ":se3_proto",
    ],
)

cc_proto_library(
    name = "fse3_proto_cc",
    visibility = ["//visibility:public"],
    deps = [":fse3_proto"],
)

proto_library(
    name = "fso3_proto",
    srcs = ["fso3.proto"],
    visibility = ["//visibility:public"],
    deps = [
        ":frame_3_proto",
        ":so3_proto",
    ],
)

cc_library(
    name = "fse3_to_proto",
    srcs = ["fse3_to_proto.cc"],
    hdrs = ["fse3_to_proto.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":framed_group_to_proto",
        ":fse3_proto_cc",
        "//resim_core/transforms:framed_group",
    ],
)

cc_proto_library(
    name = "fso3_proto_cc",
    visibility = ["//visibility:public"],
    deps = [":fso3_proto"],
)

proto_library(
    name = "so3_proto",
    srcs = ["so3.proto"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "fso3_to_proto",
    srcs = ["fso3_to_proto.cc"],
    hdrs = ["fso3_to_proto.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":framed_group_to_proto",
        ":fso3_proto_cc",
        "//resim_core/transforms:framed_group",
        "//resim_core/transforms:so3",
    ],
)

cc_proto_library(
    name = "so3_proto_cc",
    visibility = ["//visibility:public"],
    deps = [":so3_proto"],
)

cc_library(
    name = "so3_to_proto",
    srcs = ["so3_to_proto.cc"],
    hdrs = ["so3_to_proto.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":liegroup_to_proto",
        ":so3_proto_cc",
        "//resim_core/transforms:so3",
    ],
)

proto_library(
    name = "se3_proto",
    srcs = ["se3.proto"],
    visibility = ["//visibility:public"],
)

cc_proto_library(
    name = "se3_proto_cc",
    visibility = ["//visibility:public"],
    deps = [":se3_proto"],
)

cc_library(
    name = "se3_to_proto",
    srcs = ["se3_to_proto.cc"],
    hdrs = ["se3_to_proto.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":liegroup_to_proto",
        ":se3_proto_cc",
        "//resim_core/transforms:se3",
    ],
)
