load("@rules_cc//cc:defs.bzl", "cc_binary")

cc_binary(
    name = "parse_metrics",
    srcs = ["parse_metrics.cc"],
    deps = [
        "//resim/assert",
        "//resim/metrics:simple_metric",
        "//resim/metrics/proto:simple_metric_proto_cc",
        "//resim/metrics/proto:simple_metric_to_proto",
        "//resim/third_party/mcap:mcap_impl",
        "//resim/time:timestamp",
        "@cxxopts",
        "@mcap",
        "@nlohmann_json//:json",
    ],
)
