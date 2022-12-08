load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

# Gtest V 1.11
http_archive(
    name = "com_google_googletest",
    sha256 = "8daa1a71395892f7c1ec5f7cb5b099a02e606be720d62f1a6a98f8f8898ec826",
    strip_prefix = "googletest-e2239ee6043f73722e7aa812a459f54a28552929",
    urls = ["https://github.com/google/googletest/archive/e2239ee6043f73722e7aa812a459f54a28552929.zip"],
)

# Eigen V 3.3.9
http_archive(
    name = "libeigen",
    build_file = "//resim_core/third_party/eigen:eigen.BUILD",
    sha256 = "83709a8def0d60dc4d17a749989893ea5e5aacf13f9184ae0509313f400f6f45",
    strip_prefix = "eigen-3.3.9",
    urls = ["https://gitlab.com/libeigen/eigen/-/archive/3.3.9/eigen-3.3.9.zip"],
)

# Rules foreign CC
http_archive(
    name = "rules_foreign_cc",
    sha256 = "5303e3363fe22cbd265c91fce228f84cf698ab0f98358ccf1d95fba227b308f6",
    strip_prefix = "rules_foreign_cc-0.9.0",
    urls = ["https://github.com/bazelbuild/rules_foreign_cc/archive/refs/tags/0.9.0.zip"],
)

load("@rules_foreign_cc//foreign_cc:repositories.bzl", "rules_foreign_cc_dependencies")

rules_foreign_cc_dependencies()

# lz4, which is a dependency for mcap
http_archive(
    name = "lz4",
    build_file = "//resim_core/third_party/lz4:lz4.BUILD",
    sha256 = "4ec935d99aa4950eadfefbd49c9fad863185ac24c32001162c44a683ef61b580",
    strip_prefix = "lz4-1.9.3",
    urls = ["https://github.com/lz4/lz4/archive/refs/tags/v1.9.3.zip"],
)

# zstd, which is a dependency for mcap
http_archive(
    name = "zstd",
    build_file = "//resim_core/third_party/zstd:zstd.BUILD",
    sha256 = "53f4696f3cec8703f12d3402707a6aaf7eb92d43c90d61e1d32454bda5da7b9c",
    strip_prefix = "zstd-1.5.2",
    urls = ["https://github.com/facebook/zstd/archive/refs/tags/v1.5.2.zip"],
)

http_archive(
    name = "mcap",
    build_file = "//resim_core/third_party/mcap:mcap.BUILD",
    sha256 = "5d30a67c0c282e478e9342127129c3b4138c1464f42c25ba083415ced0824437",
    strip_prefix = "mcap-releases-cpp-v0.5.0",
    urls = ["https://github.com/foxglove/mcap/archive/refs/tags/releases/cpp/v0.5.0.zip"],
)

# Protobuf schemas for communications with foxglove
http_archive(
    name = "foxglove_schemas",
    build_file = "//resim_core/third_party/foxglove_schemas:schemas.BUILD",
    sha256 = "817d60451b7f09314b9ccf6eafdb5a0c2f354dc219b8d5518d1f4fc5c6f52da8",
    strip_prefix = "schemas-releases-typescript-v0.7.1/schemas/proto",
    urls = ["https://github.com/foxglove/schemas/archive/refs/tags/releases/typescript/v0.7.1.zip"],
)

http_archive(
    name = "rules_python",
    sha256 = "cdf6b84084aad8f10bf20b46b77cb48d83c319ebe6458a18e9d2cebf57807cdd",
    strip_prefix = "rules_python-0.8.1",
    url = "https://github.com/bazelbuild/rules_python/archive/refs/tags/0.8.1.tar.gz",
)

load("@rules_python//python:repositories.bzl", "python_register_toolchains")

python_register_toolchains(
    name = "python3_10",
    # Available versions are listed in @rules_python//python:versions.bzl.
    # We recommend using the same version your team is already standardized on.
    python_version = "3.10",
)

# Hedron's Compile Commands Extractor for Bazel
# https://github.com/hedronvision/bazel-compile-commands-extractor
http_archive(
    name = "hedron_compile_commands",
    sha256 = "8603191949837cd01a91a0e78c32488d781de72bcbf455c9cca79ac03160c6de",
    strip_prefix = "bazel-compile-commands-extractor-d8ff4bd0142f70e0c51b11d6297e97b81136b018",
    url = "https://github.com/hedronvision/bazel-compile-commands-extractor/archive/d8ff4bd0142f70e0c51b11d6297e97b81136b018.tar.gz",
)

load("@hedron_compile_commands//:workspace_setup.bzl", "hedron_compile_commands_setup")

hedron_compile_commands_setup()

# Protobuf
# rules_cc defines rules for generating C++ code from Protocol Buffers.
http_archive(
    name = "rules_proto",
    sha256 = "e017528fd1c91c5a33f15493e3a398181a9e821a804eb7ff5acdd1d2d6c2b18d",
    strip_prefix = "rules_proto-4.0.0-3.20.0",
    urls = [
        "https://github.com/bazelbuild/rules_proto/archive/refs/tags/4.0.0-3.20.0.tar.gz",
    ],
)

load("@rules_proto//proto:repositories.bzl", "rules_proto_dependencies", "rules_proto_toolchains")

rules_proto_dependencies()

rules_proto_toolchains()

# fmt string formatting library
http_archive(
    name = "fmt",
    patch_cmds = [
        "mv support/bazel/.bazelrc .bazelrc",
        "mv support/bazel/.bazelversion .bazelversion",
        "mv support/bazel/BUILD.bazel BUILD.bazel",
        "mv support/bazel/WORKSPACE.bazel WORKSPACE.bazel",
    ],
    sha256 = "5dea48d1fcddc3ec571ce2058e13910a0d4a6bab4cc09a809d8b1dd1c88ae6f2",
    strip_prefix = "fmt-9.1.0",
    urls = ["https://github.com/fmtlib/fmt/archive/refs/tags/9.1.0.tar.gz"],
)
