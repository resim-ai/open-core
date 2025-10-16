# Copyright 2025 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
ReSim CLI repo rule definition
"""

load("@bazel_tools//tools/build_defs/repo:cache.bzl", "get_default_canonical_id")

_VERSION = "v0.29.0"

_PLATFORMS = [
    struct(
        name = "linux-amd64",
        os = "@platforms//os:linux",
        cpu = "@platforms//cpu:x86_64",
        sha256 = "fe30975f1396d4182d6bd4d297f37c2718dd4780f6d27c472958e4bc309f9ab8",
    ),
    struct(
        name = "darwin-arm64",
        os = "@platforms//os:osx",
        cpu = "@platforms//cpu:arm64",
        sha256 = "86bf59f407b17192a53e6a0e1f86c13188449bb8341232aae92a50e41b67117a",
    ),
    struct(
        name = "darwin-amd64",
        os = "@platforms//os:osx",
        cpu = "@platforms//cpu:x86_64",
        sha256 = "b0bd65d13531982ff2f4e4ae0dae61c9b8cfdf0c553cc91c0b0be266266e1ee3",
    ),
]

def _resim_cli_impl(rctx):
    # Download binaries
    for p in _PLATFORMS:
        url = "https://github.com/resim-ai/api-client/releases/download/{}/resim-{}".format(
            _VERSION,
            p.name,
        )
        rctx.download(
            url = url,
            output = "resim-{}".format(p.name),
            executable = True,
            sha256 = p.sha256,
            canonical_id = get_default_canonical_id(rctx, [url]),
        )

    build_content = [
        """load("@bazel_skylib//rules:native_binary.bzl", "native_binary")""",
    ]

    # Generate config_settings
    for p in _PLATFORMS:
        build_content.append("""
config_setting(
    name = "{name}",
    constraint_values = [
        "{os}",
        "{cpu}",
    ],
)""".format(name = p.name, os = p.os, cpu = p.cpu))

    # Generate native_binary rule
    build_content.append("""native_binary(
    name = "resim",
    src = select({""")

    for p in _PLATFORMS:
        build_content.append("""        ":{name}": "resim-{name}",""".format(name = p.name))

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
