# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Macro to help bring in direct dependencies of this workspace.
"""

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
load("@bazel_tools//tools/build_defs/repo:utils.bzl", "maybe")

def resim_core_dependencies():
    """Macro to help bring in direct dependencies of this workspace.
    """
    maybe(
        http_archive,
        name = "rules_cc",
        urls = ["https://github.com/bazelbuild/rules_cc/releases/download/0.0.8/rules_cc-0.0.8.tar.gz"],
        sha256 = "ae46b722a8b8e9b62170f83bfb040cbf12adb732144e689985a66b26410a7d6f",
        strip_prefix = "rules_cc-0.0.8",
    )

    maybe(
        http_archive,
        name = "bazel_skylib",
        sha256 = "66ffd9315665bfaafc96b52278f57c7e2dd09f5ede279ea6d39b2be471e7e3aa",
        urls = [
            "https://mirror.bazel.build/github.com/bazelbuild/bazel-skylib/releases/download/1.4.2/bazel-skylib-1.4.2.tar.gz",
            "https://github.com/bazelbuild/bazel-skylib/releases/download/1.4.2/bazel-skylib-1.4.2.tar.gz",
        ],
    )

    # Gtest V 1.11
    maybe(
        http_archive,
        name = "com_google_googletest",
        patch_args = ["-p1"],
        patches = ["@resim_open_core//resim/third_party/googletest:googletest.patch"],
        sha256 = "ffa17fbc5953900994e2deec164bb8949879ea09b411e07f215bfbb1f87f4632",
        strip_prefix = "googletest-1.13.0",
        urls = ["https://github.com/google/googletest/archive/refs/tags/v1.13.0.zip"],
    )

    # Eigen V 3.4.0
    maybe(
        http_archive,
        name = "libeigen",
        build_file = "@resim_open_core//resim/third_party/eigen:eigen.BUILD",
        sha256 = "1ccaabbfe870f60af3d6a519c53e09f3dcf630207321dffa553564a8e75c4fc8",
        strip_prefix = "eigen-3.4.0",
        urls = ["https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.zip"],
    )

    # Rules foreign CC
    maybe(
        http_archive,
        name = "rules_foreign_cc",
        sha256 = "5303e3363fe22cbd265c91fce228f84cf698ab0f98358ccf1d95fba227b308f6",
        strip_prefix = "rules_foreign_cc-0.9.0",
        urls = ["https://github.com/bazelbuild/rules_foreign_cc/archive/refs/tags/0.9.0.zip"],
    )

    # lz4, which is a dependency for mcap
    maybe(
        http_archive,
        name = "lz4",
        build_file = "@resim_open_core//resim/third_party/lz4:lz4.BUILD",
        sha256 = "4ec935d99aa4950eadfefbd49c9fad863185ac24c32001162c44a683ef61b580",
        strip_prefix = "lz4-1.9.3",
        urls = ["https://github.com/lz4/lz4/archive/refs/tags/v1.9.3.zip"],
    )

    # zstd, which is a dependency for mcap
    maybe(
        http_archive,
        name = "zstd",
        build_file = "@resim_open_core//resim/third_party/zstd:zstd.BUILD",
        sha256 = "53f4696f3cec8703f12d3402707a6aaf7eb92d43c90d61e1d32454bda5da7b9c",
        strip_prefix = "zstd-1.5.2",
        urls = ["https://github.com/facebook/zstd/archive/refs/tags/v1.5.2.zip"],
    )

    maybe(
        http_archive,
        name = "mcap",
        build_file = "@resim_open_core//resim/third_party/mcap:mcap.BUILD",
        sha256 = "5d30a67c0c282e478e9342127129c3b4138c1464f42c25ba083415ced0824437",
        strip_prefix = "mcap-releases-cpp-v0.5.0",
        urls = ["https://github.com/foxglove/mcap/archive/refs/tags/releases/cpp/v0.5.0.zip"],
    )

    maybe(
        http_archive,
        name = "rules_python",
        sha256 = "cdf6b84084aad8f10bf20b46b77cb48d83c319ebe6458a18e9d2cebf57807cdd",
        strip_prefix = "rules_python-0.8.1",
        url = "https://github.com/bazelbuild/rules_python/archive/refs/tags/0.8.1.tar.gz",
    )

    # Hedron's Compile Commands Extractor for Bazel
    # https://github.com/hedronvision/bazel-compile-commands-extractor
    maybe(
        http_archive,
        name = "hedron_compile_commands",
        sha256 = "8fd0e0c0b0c10dde492bb9fe8bc5c313c40333a7d7ea74891e211a6e680170aa",
        strip_prefix = "bazel-compile-commands-extractor-d3afb5dfadd4beca48bb027112d029f2d34ff0a0",
        url = "https://github.com/hedronvision/bazel-compile-commands-extractor/archive/d3afb5dfadd4beca48bb027112d029f2d34ff0a0.zip",
    )

    # Protobuf
    # rules_cc defines rules for generating C++ code from Protocol Buffers.
    maybe(
        http_archive,
        name = "rules_proto",
        sha256 = "e017528fd1c91c5a33f15493e3a398181a9e821a804eb7ff5acdd1d2d6c2b18d",
        strip_prefix = "rules_proto-4.0.0-3.20.0",
        urls = [
            "https://github.com/bazelbuild/rules_proto/archive/refs/tags/4.0.0-3.20.0.tar.gz",
        ],
    )

    # fmt string formatting library
    maybe(
        http_archive,
        name = "fmt",
        build_file = "@resim_open_core//resim/third_party/fmt:fmt.BUILD",
        sha256 = "cdc885473510ae0ea909b5589367f8da784df8b00325c55c7cbbab3058424120",
        strip_prefix = "fmt-9.1.0",
        urls = ["https://github.com/fmtlib/fmt/archive/refs/tags/9.1.0.zip"],
    )

    # bazel pkg rules
    maybe(
        http_archive,
        name = "rules_pkg",
        sha256 = "eea0f59c28a9241156a47d7a8e32db9122f3d50b505fae0f33de6ce4d9b61834",
        urls = [
            "https://mirror.bazel.build/github.com/bazelbuild/rules_pkg/releases/download/0.8.0/rules_pkg-0.8.0.tar.gz",
            "https://github.com/bazelbuild/rules_pkg/releases/download/0.8.0/rules_pkg-0.8.0.tar.gz",
        ],
    )

    maybe(
        http_archive,
        name = "httplib",
        build_file = "@resim_open_core//resim/third_party/httplib:httplib.BUILD",
        sha256 = "9d884bee50ded17de3a51d673bb9c2d7d11f101c595b0891830ba9d02e0cc368",
        strip_prefix = "cpp-httplib-0.11.4",
        urls = ["https://github.com/yhirose/cpp-httplib/archive/refs/tags/v0.11.4.zip"],
    )

    # Glog
    maybe(
        http_archive,
        name = "com_github_gflags_gflags",
        sha256 = "34af2f15cf7367513b352bdcd2493ab14ce43692d2dcd9dfc499492966c64dcf",
        strip_prefix = "gflags-2.2.2",
        urls = ["https://github.com/gflags/gflags/archive/v2.2.2.tar.gz"],
    )

    maybe(
        http_archive,
        name = "com_github_google_glog",
        sha256 = "122fb6b712808ef43fbf80f75c52a21c9760683dae470154f02bddfc61135022",
        strip_prefix = "glog-0.6.0",
        urls = ["https://github.com/google/glog/archive/v0.6.0.zip"],
    )

    # Polygon Mesh Processing (pmp) library. Used for loading, manipulating, and saving meshes.
    maybe(
        http_archive,
        name = "pmp",
        build_file = "@resim_open_core//resim/third_party/pmp:pmp.BUILD",
        sha256 = "8ff56706694bfc85ae1cbbf0777b347d1f2e48d04863b86d73ec42c919ca7fd2",
        strip_prefix = "pmp-library-2.0.1",
        urls = ["https://github.com/pmp-library/pmp-library/archive/refs/tags/2.0.1.zip"],
    )

    # libccd - Used by fcl (below) for narrow-phase collision detection.
    maybe(
        http_archive,
        name = "libccd",
        build_file = "@resim_open_core//resim/third_party/libccd:libccd.BUILD",
        patch_args = ["-p1"],
        # Patch to generate files ordinarily generated by cmake:
        patches = ["@resim_open_core//resim/third_party/libccd:libccd.patch"],
        sha256 = "332e9b14eee970f1afb3cfc7ae7233013e448266fd2caefe1b9332581f823e8d",
        strip_prefix = "libccd-2.1",
        urls = ["https://github.com/danfis/libccd/archive/refs/tags/v2.1.zip"],
    )

    # Flexible Collision Library (fcl). Used for efficient broad-phase collision detection.
    maybe(
        http_archive,
        name = "fcl",
        build_file = "@resim_open_core//resim/third_party/fcl:fcl.BUILD",
        patch_args = ["-p1"],
        # Patch to generate files ordinarily generated by cmake:
        patches = ["@resim_open_core//resim/third_party/fcl:fcl.patch"],
        sha256 = "767dd87f704a6400e7fef391fc450f7975d28846394e17d81b8105593c323df7",
        strip_prefix = "fcl-0.7.0",
        urls = ["https://github.com/flexible-collision-library/fcl/archive/refs/tags/0.7.0.zip"],
    )

    maybe(
        http_archive,
        name = "au",
        sha256 = "4709343cbdae957c3f00d04d7523c62ee6be94305baa3007750c23fb5050fc29",
        strip_prefix = "au-0.3.0",
        urls = ["https://github.com/aurora-opensource/au/archive/refs/tags/0.3.0.zip"],
    )

    maybe(
        http_archive,
        name = "flamegraph",
        build_file = "@resim_open_core//resim/third_party/flamegraph:flamegraph.BUILD",
        sha256 = "831673a8a130792cd8ad0f2300d3e420dde7430d50d9ae5c3985f03f86d3e7db",
        strip_prefix = "FlameGraph-1.0.0",
        urls = ["https://github.com/resim-ai/FlameGraph/archive/refs/tags/v1.0.0.zip"],
    )

    maybe(
        http_archive,
        name = "indicators",
        build_file = "@resim_open_core//resim/third_party/indicators:indicators.BUILD",
        sha256 = "c1deca4b72d655ba457882a5d1b75011f99253c6a2cc8d2f5bc7c49324b2e4ff",
        strip_prefix = "indicators-2.3",
        urls = ["https://github.com/p-ranav/indicators/archive/refs/tags/v2.3.zip"],
    )

    # Dependency for cxxopts below
    maybe(
        http_archive,
        name = "rules_fuzzing",
        sha256 = "d9002dd3cd6437017f08593124fdd1b13b3473c7b929ceb0e60d317cb9346118",
        strip_prefix = "rules_fuzzing-0.3.2",
        urls = ["https://github.com/bazelbuild/rules_fuzzing/archive/v0.3.2.zip"],
    )

    maybe(
        http_archive,
        name = "cxxopts",
        sha256 = "25b644a2bfa9c6704d723be51b026bc02420dfdee1277a49bfe5df3f19b0eaa4",
        strip_prefix = "cxxopts-3.1.1",
        urls = ["https://github.com/jarro2783/cxxopts/archive/refs/tags/v3.1.1.zip"],
    )

    # Hedron's Bazel Rules for C++ HTTPS Requests
    # Makes @cpr, @curl, and @boringssl available for use
    # https://github.com/hedronvision/bazel-make-cc-https-easy
    maybe(
        http_archive,
        name = "hedron_make_cc_https_easy",
        sha256 = "85488c2db684db127c867bb6b5a47e47f844140fc6c64047895d1d19c47e3006",
        strip_prefix = "bazel-make-cc-https-easy-f4f7512af5644f6f37e2270e0d36debad2bde45d",
        url = "https://github.com/hedronvision/bazel-make-cc-https-easy/archive/f4f7512af5644f6f37e2270e0d36debad2bde45d.tar.gz",
    )

    maybe(
        http_archive,
        name = "nlohmann_json",
        # This build file is required because bazel BUILD support is in a version *after* the latest release...
        build_file = "@resim_open_core//resim/third_party/nlohmann_json:nlohmann_json.BUILD",
        sha256 = "95651d7d1fcf2e5c3163c3d37df6d6b3e9e5027299e6bd050d157322ceda9ac9",
        strip_prefix = "json-3.11.2",
        url = "https://github.com/nlohmann/json/archive/refs/tags/v3.11.2.zip",
    )

    maybe(
        http_archive,
        name = "com_github_nelhage_rules_boost",
        url = "https://github.com/nelhage/rules_boost/archive/96e9b631f104b43a53c21c87b01ac538ad6f3b48.tar.gz",
        sha256 = "5ea00abc70cdf396a23fb53201db19ebce2837d28887a08544429d27783309ed",
        strip_prefix = "rules_boost-96e9b631f104b43a53c21c87b01ac538ad6f3b48",
    )

    maybe(
        http_archive,
        name = "com_github_mvukov_rules_ros2",
        patch_args = ["-p1"],
        patches = [
            "@resim_open_core//resim/third_party/ros2:flto.patch",
            "@resim_open_core//resim/third_party/ros2:copts.patch",
        ],
        sha256 = "56e78c7910c6684a051c0754cc58739484800b11a64e542194a25c34dc8bc488",
        strip_prefix = "rules_ros2-e5ca0b2f86a3e9f1b97ff8d6b2a8e3e03185ad81",
        url = "https://github.com/mvukov/rules_ros2/archive/e5ca0b2f86a3e9f1b97ff8d6b2a8e3e03185ad81.tar.gz",
    )

    maybe(
        http_archive,
        name = "vision_msgs",
        sha256 = "7b9f94e65e5228d138fc097856bd3fa9c1e731d0cb1a373f5e47e94addd136b0",
        build_file = "@resim_open_core//resim/third_party/vision_msgs:vision_msgs.BUILD",
        strip_prefix = "vision_msgs-4.1.0/vision_msgs/msg",
        url = "https://github.com/ros-perception/vision_msgs/archive/refs/tags/4.1.0.zip",
    )
