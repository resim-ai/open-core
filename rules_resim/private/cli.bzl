# Copyright 2025 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
ReSim CLI repo rule definition
"""

_PLATFORMS = [
    ("linux-amd64", "@platforms//os:linux", "@platforms//cpu:x86_64"),
    ("darwin-arm64", "@platforms//os:osx", "@platforms//cpu:arm64"),
    ("darwin-amd64", "@platforms//os:osx", "@platforms//cpu:x86_64"),
]

def _resim_cli_impl(rctx):
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
    doc = """Repo rule for fetching the resim CLI.

This repository rule downloads prebuilt `resim` CLI binaries for supported
platforms (Linux and macOS, both x86_64 and arm64). It then exposes a
`resim` target as a native binary, selectable by the host platform.

For example, if you use this rule in your `MODULE.bazel`:
```
resim_cli = use_extension("@rules_resim//:extensions.bzl", "resim_cli_extension")
use_repo(resim_cli, "resim_cli")
```

You can run the CLI like so.
```
bazel run @resim_cli//:resim
```
""",
)
