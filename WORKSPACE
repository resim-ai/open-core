load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

# Go rules
http_archive(
    name = "io_bazel_rules_go",
    sha256 = "56d8c5a5c91e1af73eca71a6fab2ced959b67c86d12ba37feedb0a2dfea441a6",
    urls = [
        "https://mirror.bazel.build/github.com/bazelbuild/rules_go/releases/download/v0.37.0/rules_go-v0.37.0.zip",
        "https://github.com/bazelbuild/rules_go/releases/download/v0.37.0/rules_go-v0.37.0.zip",
    ],
)

# Gazelle, for autogenerating Go BUILD files.
http_archive(
    name = "bazel_gazelle",
    sha256 = "448e37e0dbf61d6fa8f00aaa12d191745e14f07c31cabfa731f0c8e8a4f41b97",
    urls = [
        "https://mirror.bazel.build/github.com/bazelbuild/bazel-gazelle/releases/download/v0.28.0/bazel-gazelle-v0.28.0.tar.gz",
        "https://github.com/bazelbuild/bazel-gazelle/releases/download/v0.28.0/bazel-gazelle-v0.28.0.tar.gz",
    ],
)

load("@bazel_gazelle//:deps.bzl", "gazelle_dependencies")
load("@io_bazel_rules_go//go:deps.bzl", "go_register_toolchains", "go_rules_dependencies")

go_rules_dependencies()

go_register_toolchains(version = "1.19.3")

gazelle_dependencies()

# Gtest V 1.11
http_archive(
    name = "com_google_googletest",
    sha256 = "8daa1a71395892f7c1ec5f7cb5b099a02e606be720d62f1a6a98f8f8898ec826",
    strip_prefix = "googletest-e2239ee6043f73722e7aa812a459f54a28552929",
    urls = ["https://github.com/google/googletest/archive/e2239ee6043f73722e7aa812a459f54a28552929.zip"],
)

# Eigen V 3.4.0
http_archive(
    name = "libeigen",
    build_file = "//resim_core/third_party/eigen:eigen.BUILD",
    sha256 = "1ccaabbfe870f60af3d6a519c53e09f3dcf630207321dffa553564a8e75c4fc8",
    strip_prefix = "eigen-3.4.0",
    urls = ["https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.zip"],
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
    sha256 = "8fd0e0c0b0c10dde492bb9fe8bc5c313c40333a7d7ea74891e211a6e680170aa",
    strip_prefix = "bazel-compile-commands-extractor-d3afb5dfadd4beca48bb027112d029f2d34ff0a0",
    url = "https://github.com/hedronvision/bazel-compile-commands-extractor/archive/d3afb5dfadd4beca48bb027112d029f2d34ff0a0.zip",
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
    build_file = "//resim_core/third_party/fmt:fmt.BUILD",
    sha256 = "cdc885473510ae0ea909b5589367f8da784df8b00325c55c7cbbab3058424120",
    strip_prefix = "fmt-9.1.0",
    urls = ["https://github.com/fmtlib/fmt/archive/refs/tags/9.1.0.zip"],
)

# libcurl
http_archive(
    name = "curl",
    build_file = "//resim_core/third_party/curl:curl.BUILD",
    sha256 = "6147ac0b22f8c11cbd3933d7fec064dee373402c3705193ceb703a5a665f2e0c",
    strip_prefix = "curl-7.87.0",
    urls = ["https://github.com/curl/curl/releases/download/curl-7_87_0/curl-7.87.0.zip"],
)

http_archive(
    name = "httplib",
    build_file = "//resim_core/third_party/httplib:httplib.BUILD",
    sha256 = "9d884bee50ded17de3a51d673bb9c2d7d11f101c595b0891830ba9d02e0cc368",
    strip_prefix = "cpp-httplib-0.11.4",
    urls = ["https://github.com/yhirose/cpp-httplib/archive/refs/tags/v0.11.4.zip"],
)

# Glog
http_archive(
    name = "com_github_gflags_gflags",
    sha256 = "34af2f15cf7367513b352bdcd2493ab14ce43692d2dcd9dfc499492966c64dcf",
    strip_prefix = "gflags-2.2.2",
    urls = ["https://github.com/gflags/gflags/archive/v2.2.2.tar.gz"],
)

http_archive(
    name = "com_github_google_glog",
    sha256 = "122fb6b712808ef43fbf80f75c52a21c9760683dae470154f02bddfc61135022",
    strip_prefix = "glog-0.6.0",
    urls = ["https://github.com/google/glog/archive/v0.6.0.zip"],
)
