"""Repo rule for fetching the resim CLI."""

_PLATFORMS = [
    ("linux-amd64", "@platforms//os:linux", "@platforms//cpu:x86_64"),
    ("darwin-arm64", "@platforms//os:osx", "@platforms//cpu:arm64"),
    ("darwin-amd64", "@platforms//os:osx", "@platforms//cpu:x86_64"),
]

def _resim_cli_impl(rctx):
    print(rctx)
    for p, _, _ in _PLATFORMS:
        rctx.download(
            url = "https://github.com/resim-ai/api-client/releases/latest/download/resim-{}".format(p),
            output = "resim-{}".format(p),
            executable = True,
        )

    build_content = [
        """load("@bazel_skylib//rules:native_binary.bzl", "native_binary")""",
    ]
    for p, os, cpu in _PLATFORMS:
        build_content.append("""
config_setting(
     name = "{p}",
     constraint_values = [
        "{os}",
        "{cpu}",
     ],
)""".format(p = p, os = os, cpu = cpu))

    build_content.append("""native_binary(
    name = "resim",
    src = select({""")

    for p, _, _ in _PLATFORMS:
        build_content.append("""        ":{p}": "resim-{p}",""".format(p = p))

    build_content.append("""    }),
    out = "resim",
    visibility = ["//visibility:public"],
)""")

    rctx.file("BUILD.bazel", "\n".join(build_content))

resim_cli = repository_rule(
    implementation = _resim_cli_impl,
)
