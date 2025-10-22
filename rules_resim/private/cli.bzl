# Copyright 2025 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
ReSim CLI repo rule definition
"""

load("@bazel_tools//tools/build_defs/repo:cache.bzl", "get_default_canonical_id")

def _resim_cli_platform_impl(rctx):
    url = "https://github.com/resim-ai/api-client/releases/download/{}/resim-{}".format(
        rctx.attr.version,
        rctx.attr.binary_name,
    )
    rctx.download(
        url = url,
        output = "resim-{}".format(rctx.attr.binary_name),
        executable = True,
        canonical_id = get_default_canonical_id(rctx, [url]),
        sha256 = rctx.attr.sha256,
    )
    rctx.file("BUILD.bazel", """load("@bazel_skylib//rules:native_binary.bzl", "native_binary")
native_binary(
    name = "resim",
    src = "resim-{}",
    visibility = ["//visibility:public"],
)
""".format(rctx.attr.binary_name))

resim_cli_platform = repository_rule(
    implementation = _resim_cli_platform_impl,
    attrs = {
        "binary_name": attr.string(mandatory = True),
        "sha256": attr.string(mandatory = True, doc = "The checksum to use when fetching the cli"),
        "version": attr.string(mandatory = True, doc = "The CLI release version (e.g. v0.29.0)"),
        "_cli_versions": attr.label(
            allow_single_file = True,
            default = ":cli_versions.json",
        ),
    },
    doc = """Repo rule for pulling the resim cli for a specific version and platform.""",
)

def _resim_cli_impl(rctx):
    rctx.file("defs.bzl", "RESIM_CLI_VERSION = \"{}\"".format(rctx.attr.version))

    # Load our JSON file with the platforms and checksums for the binaries for each version
    platforms_by_version = json.decode(rctx.read(rctx.path(rctx.attr._cli_versions)))
    platforms = [struct(**p) for p in platforms_by_version[rctx.attr.version]]

    build_content = ["""load("@bazel_skylib//:bzl_library.bzl", "bzl_library")    
bzl_library(
    name = "defs",
    srcs = [":defs.bzl"],
    visibility = ["//visibility:public"],
)"""]

    # Generate config_settings
    for p in platforms:
        build_content.append("""
config_setting(
    name = "{name}",
    constraint_values = [
        "{os}",
        "{cpu}",
    ],
)""".format(name = p.name, os = p.os, cpu = p.cpu))

    # Generate an alias to the correct platform
    build_content.append("""alias(
    name = "resim",
    actual = select({""")

    for p in platforms:
        build_content.append("""        ":{name}": "@resim_cli_{name}//:resim",""".format(name = p.name))

    build_content.append("""    }),
    visibility = ["//visibility:public"],
)""")

    rctx.file("BUILD.bazel", "\n".join(build_content))

_resim_cli = repository_rule(
    implementation = _resim_cli_impl,
    doc = """Repo rule for resim cli interface repo
This repository rule creates an interface repo that contains a `@repo_name//:resim` alias that uses
a select() to get the right platform specific repo (defined by resim_cli_platform) for the current
target. We delegate to these repos to fetch the cli for different versions so we can fetch lazily.""",
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

def resim_cli(name, version, platforms):
    """Macro with repo rule for fetching the resim CLI.

    This macro creates repos to download prebuilt `resim` CLI binaries for supported
    platforms (Linux and macOS, both x86_64 and arm64). It then exposes a
    `resim` target as a native binary in an interface repo, selectable by the host platform.

    Args:
      name: The name of the repo to create for the resim cli.
      version: The version of the resim cli to fetch.
      platforms: The platform dicts containing name, platform, and sha information for this version.
    """
    for p_dict in platforms:
        p = struct(**p_dict)
        resim_cli_platform(
            name = "{}_{}".format(name, p.name),
            binary_name = p.name,
            sha256 = p.sha256,
            version = version,
        )
    _resim_cli(name = name, version = version)
