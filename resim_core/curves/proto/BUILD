load("@rules_cc//cc:defs.bzl", "cc_library", "cc_proto_library", "cc_test")
load("@rules_proto//proto:defs.bzl", "proto_library")

proto_library(
    name = "d_curve_proto",
    srcs = ["d_curve.proto"],
    deps = [
        "//resim_core/transforms/proto:fse3_proto",
        "//resim_core/transforms/proto:se3_proto",
    ],
)

cc_proto_library(
    name = "d_curve_proto_cc",
    visibility = ["//visibility:public"],
    deps = [":d_curve_proto"],
)

cc_library(
    name = "d_curve_to_proto",
    srcs = ["d_curve_to_proto.cc"],
    hdrs = ["d_curve_to_proto.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":d_curve_proto_cc",
        "//resim_core/curves:d_curve",
        "//resim_core/transforms:se3",
        "//resim_core/transforms/proto:fse3_to_proto",
        "//resim_core/transforms/proto:se3_to_proto",
        "//resim_core/utils:inout",
        "@com_github_google_glog//:glog",
    ],
)

cc_library(
    name = "d_curve_se3_to_proto",
    srcs = ["d_curve_se3_to_proto.cc"],
    hdrs = ["d_curve_se3_to_proto.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":d_curve_proto_cc",
        ":d_curve_to_proto",
        "//resim_core/transforms:se3",
        "//resim_core/utils:inout",
    ],
)

cc_library(
    name = "d_curve_fse3_to_proto",
    srcs = ["d_curve_fse3_to_proto.cc"],
    hdrs = ["d_curve_fse3_to_proto.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":d_curve_proto_cc",
        ":d_curve_to_proto",
        "//resim_core/curves:d_curve",
        "//resim_core/transforms:framed_group",
        "//resim_core/utils:inout",
    ],
)

cc_test(
    name = "d_curve_to_proto_test",
    srcs = ["d_curve_to_proto_test.cc"],
    deps = [
        ":d_curve_fse3_to_proto",
        ":d_curve_proto_cc",
        ":d_curve_se3_to_proto",
        ":d_curve_to_proto",
        "//resim_core/curves:d_curve",
        "//resim_core/transforms:liegroup_test_helpers",
        "//resim_core/transforms:se3",
        "//resim_core/transforms/proto:fse3_to_proto",
        "//resim_core/transforms/proto:se3_to_proto",
        "//resim_core/utils:inout",
        "@com_google_googletest//:gtest_main",
    ],
)

proto_library(
    name = "two_jet_proto",
    srcs = ["two_jet.proto"],
    deps = [
        "//resim_core/transforms/proto:fse3_proto",
        "//resim_core/transforms/proto:fso3_proto",
        "//resim_core/transforms/proto:se3_proto",
        "//resim_core/transforms/proto:so3_proto",
    ],
)

cc_proto_library(
    name = "two_jet_proto_cc",
    visibility = ["//visibility:public"],
    deps = [":two_jet_proto"],
)

cc_library(
    name = "two_jet_to_proto",
    srcs = ["two_jet_to_proto.cc"],
    hdrs = ["two_jet_to_proto.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":two_jet_proto_cc",
        "//resim_core/curves:two_jet",
        "//resim_core/math/proto:matrix_to_proto",
        "//resim_core/transforms:framed_group",
        "//resim_core/transforms:se3",
        "//resim_core/transforms:so3",
        "//resim_core/transforms/proto:fse3_to_proto",
        "//resim_core/transforms/proto:fso3_to_proto",
        "//resim_core/transforms/proto:se3_to_proto",
        "//resim_core/transforms/proto:so3_to_proto",
        "//resim_core/utils:inout",
    ],
)

cc_library(
    name = "two_jetl_se3_to_proto",
    srcs = ["two_jetl_se3_to_proto.cc"],
    hdrs = ["two_jetl_se3_to_proto.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":two_jet_proto_cc",
        ":two_jet_to_proto",
        "//resim_core/curves:two_jet",
        "//resim_core/transforms:framed_group",
    ],
)

cc_library(
    name = "two_jetl_so3_to_proto",
    srcs = ["two_jetl_so3_to_proto.cc"],
    hdrs = ["two_jetl_so3_to_proto.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":two_jet_proto_cc",
        ":two_jet_to_proto",
        "//resim_core/curves:two_jet",
        "//resim_core/transforms:framed_group",
    ],
)

cc_library(
    name = "two_jetl_fso3_to_proto",
    srcs = ["two_jetl_fso3_to_proto.cc"],
    hdrs = ["two_jetl_fso3_to_proto.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":two_jet_proto_cc",
        ":two_jet_to_proto",
        "//resim_core/curves:two_jet",
        "//resim_core/transforms:framed_group",
    ],
)

cc_library(
    name = "two_jetl_fse3_to_proto",
    srcs = ["two_jetl_fse3_to_proto.cc"],
    hdrs = ["two_jetl_fse3_to_proto.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":two_jet_proto_cc",
        ":two_jet_to_proto",
        "//resim_core/curves:two_jet",
        "//resim_core/transforms:framed_group",
    ],
)

cc_library(
    name = "two_jetr_se3_to_proto",
    srcs = ["two_jetr_se3_to_proto.cc"],
    hdrs = ["two_jetr_se3_to_proto.hh"],
    deps = [
        ":two_jet_proto_cc",
        ":two_jet_to_proto",
        "//resim_core/curves:two_jet",
        "//resim_core/transforms:framed_group",
    ],
)

cc_library(
    name = "two_jetr_so3_to_proto",
    srcs = ["two_jetr_so3_to_proto.cc"],
    hdrs = ["two_jetr_so3_to_proto.hh"],
    deps = [
        ":two_jet_proto_cc",
        ":two_jet_to_proto",
        "//resim_core/curves:two_jet",
        "//resim_core/transforms:framed_group",
    ],
)

cc_library(
    name = "two_jetr_fso3_to_proto",
    srcs = ["two_jetr_fso3_to_proto.cc"],
    hdrs = ["two_jetr_fso3_to_proto.hh"],
    deps = [
        ":two_jet_proto_cc",
        ":two_jet_to_proto",
        "//resim_core/curves:two_jet",
        "//resim_core/transforms:framed_group",
    ],
)

cc_library(
    name = "two_jetr_fse3_to_proto",
    srcs = ["two_jetr_fse3_to_proto.cc"],
    hdrs = ["two_jetr_fse3_to_proto.hh"],
    deps = [
        ":two_jet_proto_cc",
        ":two_jet_to_proto",
        "//resim_core/curves:two_jet",
        "//resim_core/transforms:framed_group",
    ],
)

cc_test(
    name = "two_jet_to_proto_test",
    srcs = ["two_jet_to_proto_test.cc"],
    deps = [
        ":two_jet_proto_cc",
        ":two_jet_to_proto",
        ":two_jetl_fse3_to_proto",
        ":two_jetl_fso3_to_proto",
        ":two_jetl_se3_to_proto",
        ":two_jetl_so3_to_proto",
        ":two_jetr_fse3_to_proto",
        ":two_jetr_fso3_to_proto",
        ":two_jetr_se3_to_proto",
        ":two_jetr_so3_to_proto",
        "//resim_core/curves:two_jet_test_helpers",
        "//resim_core/math/proto:matrix_to_proto",
        "//resim_core/testing:random_matrix",
        "//resim_core/transforms:framed_group",
        "//resim_core/transforms:framed_group_concept",
        "//resim_core/transforms:se3",
        "//resim_core/transforms:so3",
        "//resim_core/transforms/proto:fse3_to_proto",
        "//resim_core/transforms/proto:fso3_to_proto",
        "//resim_core/transforms/proto:se3_to_proto",
        "//resim_core/transforms/proto:so3_to_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

proto_library(
    name = "t_curve_proto",
    srcs = ["t_curve.proto"],
    visibility = ["//visibility:public"],
    deps = [":two_jet_proto"],
)

cc_proto_library(
    name = "t_curve_proto_cc",
    visibility = ["//visibility:public"],
    deps = [":t_curve_proto"],
)

cc_library(
    name = "t_curve_to_proto",
    srcs = ["t_curve_to_proto.cc"],
    hdrs = ["t_curve_to_proto.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":t_curve_proto_cc",
        ":two_jet_to_proto",
        ":two_jetl_fse3_to_proto",
        ":two_jetl_fso3_to_proto",
        ":two_jetl_se3_to_proto",
        ":two_jetl_so3_to_proto",
        "//resim_core/curves:t_curve",
        "//resim_core/transforms:framed_group",
        "//resim_core/transforms:se3",
        "//resim_core/transforms:so3",
        "//resim_core/utils:inout",
    ],
)

cc_library(
    name = "t_curve_fse3_to_proto",
    srcs = ["t_curve_fse3_to_proto.cc"],
    hdrs = ["t_curve_fse3_to_proto.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":t_curve_proto_cc",
        ":t_curve_to_proto",
        "//resim_core/curves:t_curve",
        "//resim_core/transforms:framed_group",
    ],
)

cc_library(
    name = "t_curve_fso3_to_proto",
    srcs = ["t_curve_fso3_to_proto.cc"],
    hdrs = ["t_curve_fso3_to_proto.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":t_curve_proto_cc",
        ":t_curve_to_proto",
        "//resim_core/curves:t_curve",
        "//resim_core/transforms:framed_group",
    ],
)

cc_library(
    name = "t_curve_se3_to_proto",
    srcs = ["t_curve_se3_to_proto.cc"],
    hdrs = ["t_curve_se3_to_proto.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":t_curve_proto_cc",
        ":t_curve_to_proto",
        "//resim_core/curves:t_curve",
        "//resim_core/transforms:se3",
    ],
)

cc_library(
    name = "t_curve_so3_to_proto",
    srcs = ["t_curve_so3_to_proto.cc"],
    hdrs = ["t_curve_so3_to_proto.hh"],
    visibility = ["//visibility:public"],
    deps = [
        ":t_curve_proto_cc",
        ":t_curve_to_proto",
        "//resim_core/curves:t_curve",
        "//resim_core/transforms:so3",
    ],
)

cc_test(
    name = "t_curve_to_proto_test",
    srcs = ["t_curve_to_proto_test.cc"],
    deps = [
        ":t_curve_fse3_to_proto",
        ":t_curve_fso3_to_proto",
        ":t_curve_proto_cc",
        ":t_curve_se3_to_proto",
        ":t_curve_so3_to_proto",
        ":t_curve_to_proto",
        ":two_jetl_fse3_to_proto",
        ":two_jetl_fso3_to_proto",
        ":two_jetl_se3_to_proto",
        ":two_jetl_so3_to_proto",
        "//resim_core/curves:t_curve",
        "//resim_core/curves:t_curve_test_helpers",
        "//resim_core/transforms:framed_group",
        "//resim_core/transforms:framed_group_concept",
        "@com_google_googletest//:gtest_main",
    ],
)
