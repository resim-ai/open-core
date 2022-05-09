load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

# Gtest V 1.11
http_archive(
    name = "com_google_googletest",
    strip_prefix = "googletest-e2239ee6043f73722e7aa812a459f54a28552929",
    urls = ["https://github.com/google/googletest/archive/e2239ee6043f73722e7aa812a459f54a28552929.zip"],
)

# Eigen V 3.3.9
http_archive(
    name = "libeigen",
    build_file = "//build:eigen.BUILD",
    sha256 = "83709a8def0d60dc4d17a749989893ea5e5aacf13f9184ae0509313f400f6f45",
    strip_prefix = "eigen-3.3.9",
    urls = ["https://gitlab.com/libeigen/eigen/-/archive/3.3.9/eigen-3.3.9.zip"],
)
