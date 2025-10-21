# Copyright 2025 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
ReSim CLI repo rule definition
"""

load("@bazel_tools//tools/build_defs/repo:cache.bzl", "get_default_canonical_id")

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
    rctx.file("defs.bzl", "RESIM_CLI_VERSION = \"{}\"".format(rctx.attr.version))

    # Load our JSON file with the platforms and checksums for the binaries for each version
    platforms_by_version = json.decode(rctx.read(rctx.path(rctx.attr._cli_versions)))

    # Download binaries
    for p_dict in platforms_by_version[rctx.attr.version]:
        p = struct(**p_dict)
        url = "https://github.com/resim-ai/api-client/releases/download/{}/resim-{}".format(
            rctx.attr.version,
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
    attrs = {
        "version": attr.string(mandatory = True, doc = "The CLI release version (e.g. v0.29.0)"),
        "_cli_versions": attr.label(
            allow_single_file = True,
            default = ":cli_versions.json",
        ),
    },
)

def version_to_tuple(v):
    """Convert a version number to a tuple of (major, minor, patch) (e.g. vXX.YY.ZZ -> (XX, YY, ZZ).

    Args:
        v: The version string

    Returns:
        The (major, minor, patch) tuple as integers
    """
    if not v or v[0] != "v":
        fail("Invalid version: %s" % v)
    parts = v[1:].split(".")
    if len(parts) != 3:
        fail("Invalid version: %s" % v)
    return (int(parts[0]), int(parts[1]), int(parts[2]))

def resolve_cli_version(module_ctx):
    """
    Resolves an appropriate CLI version.

    Resolve different modules' CLI versions prioritizing the root module's request and falling back
    to the highest requested version otherwise.

    Args:
        module_ctx: The module context containig modules to resolve versions over

    Returns:
        The resolved version as a string

    """

    # set() doesn't exist before bazel 8.1.0 so we use a dict
    requested_version_tuples = {}
    root_module_version = None

    for mod in module_ctx.modules:
        versions = mod.tags.versions
        if len(versions) > 1:
            fail("Only one CLI version tag allowed per module in module " + mod.name)

        if len(versions) == 1:
            tag = versions[0]
            if not tag.cli_version:
                fail("cli_version tag attribute must not be empty in module " + mod.name)

            if mod.is_root:
                root_module_version = tag.cli_version
            else:
                requested_version_tuples[version_to_tuple(tag.cli_version)] = True

    if root_module_version:
        print("resim_cli: Root module requested version: %s. Using it for override." % root_module_version)  # buildifier: disable=print
        return root_module_version

    if requested_version_tuples:
        sorted_versions = sorted(list(requested_version_tuples.keys()))
        selected_version = sorted_versions[-1]
        return "v%s.%s.%s" % selected_version

    fail("No version specified in any module ReSim for the ReSim CLI")
