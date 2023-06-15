load("@rules_cc//cc:defs.bzl", "cc_binary")
load("@rules_pkg//pkg:mappings.bzl", "pkg_filegroup", "pkg_files", "strip_prefix")
load("@rules_pkg//pkg:zip.bzl", "pkg_zip")

cc_binary(
    name = "example_00_rigid_transform",
    srcs = ["example_00_rigid_transform.cc"],
    deps = [
        "//resim/transforms:se3",
        "//resim/transforms:so3",
        "//resim/visualization:view",
    ],
)

cc_binary(
    name = "example_01_framed_transform_composition",
    srcs = ["example_01_framed_transform_composition.cc"],
    deps = [
        "//resim/transforms:se3",
        "//resim/transforms:so3",
        "//resim/visualization:view",
    ],
)

cc_binary(
    name = "example_02_distance_parameterized_unit_circle",
    srcs = ["example_02_distance_parameterized_unit_circle.cc"],
    deps = [
        "//resim/curves:d_curve",
        "//resim/transforms:se3",
        "//resim/transforms:so3",
        "//resim/visualization:view",
    ],
)

cc_binary(
    name = "example_03_time_parameterized_closed_curve",
    srcs = ["example_03_time_parameterized_closed_curve.cc"],
    deps = [
        "//resim/curves:t_curve",
        "//resim/testing:random_matrix",
        "//resim/transforms:se3",
        "//resim/transforms:so3",
        "//resim/visualization:view",
    ],
)

cc_binary(
    name = "example_04_actor_trajectory",
    srcs = ["example_04_actor_trajectory.cc"],
    deps = [
        "//resim/actor/state:rigid_body_state",
        "//resim/actor/state:trajectory",
        "//resim/time:timestamp",
        "//resim/transforms:frame",
        "//resim/transforms:se3",
        "//resim/transforms:so3",
        "//resim/utils:uuid",
        "//resim/visualization:view",
    ],
)

# Package Rules to Create Demo Binary Package:

pkg_files(
    name = "demo_headers",
    srcs = [
        "//resim/curves:d_curve.hh",
        "//resim/curves:quintic_poly_coeffs.hh",
        "//resim/curves:t_curve.hh",
        "//resim/curves:t_curve_segment.hh",
        "//resim/curves:two_jet.hh",
        "//resim/curves:two_jet_concepts.hh",
        "//resim/transforms:frame.hh",
        "//resim/transforms:liegroup.hh",
        "//resim/transforms:liegroup_concepts.hh",
        "//resim/transforms:se3.hh",
        "//resim/transforms:so3.hh",
        "//resim/utils:inout.hh",
        "//resim/utils:uuid.hh",
        "//resim/visualization:view.hh",
        "//resim/visualization:view_primitive.hh",
    ],
    prefix = "include",
    strip_prefix = strip_prefix.from_root(),
)

cc_binary(
    name = "review",
    srcs = [
        "//resim/curves:d_curve.cc",
        "//resim/curves:t_curve.cc",
        "//resim/curves:two_jet.cc",
        "//resim/transforms:se3.cc",
        "//resim/transforms:so3.cc",
        "//resim/utils:uuid.cc",
        "//resim/visualization:view.cc",
    ],
    linkshared = True,
    deps = [
        "//resim/curves:d_curve",
        "//resim/curves:t_curve",
        "//resim/curves:two_jet",
        "//resim/transforms:se3",
        "//resim/transforms:so3",
        "//resim/utils:uuid",
        "//resim/visualization:view",
    ],
)

pkg_files(
    name = "demo_libs",
    srcs = [
        ":review",
    ],
    prefix = "lib",
)

pkg_files(
    name = "demo_sources",
    srcs = [
        "example_00_rigid_transform.cc",
        "example_01_framed_transform_composition.cc",
        "example_02_distance_parameterized_unit_circle.cc",
        "example_03_time_parameterized_closed_curve.cc",
        "example_04_actor_trajectory.cc",
        "//resim/visualization/examples/package:package_files",
    ],
)

pkg_filegroup(
    name = "demo_files",
    srcs = [
        ":demo_headers",
        ":demo_libs",
        ":demo_sources",
    ],
    prefix = "review_demo",
)

pkg_zip(
    name = "demo",
    srcs = [
        ":demo_files",
    ],
    strip_prefix = ".",
)
