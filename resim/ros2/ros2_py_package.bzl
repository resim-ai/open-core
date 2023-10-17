# Copyright 2023 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""
Tools for packaging ROS2-dependent libs in python

This module defines an aspect, rules, and a macron to be used when
assembling a filetree for packaging by py_wheel in
@rules_python//python:packaging.bzl. There are a number of details
which need to be taken care of when doing this. Namely:

 - When packaging up the dynamic libraries that ROS requires, we
   cannot simply leave them at the default external or _solib
   directory where bazel puts them relative to the binaries linking
   them. This would make it possible for users to "import external" or
   "import _solib" from their python which is unacceptable. This means
   that we need to move them to a resim.libs directory AND adjust the
   runpath of all binaries to include this directory. We cannot easily
   use LD_LIBRARY_PATH to facilitate this because setting it from
   within a python process (e.g. on import in __init__.py) has no
   effect for that particular process. In order to accomplish this
   change in runpath, we use an aspect to relink these dynamic
   libraries with the runpath we would like.

 - Furthermore, the ROS libraries themselves require a couple of
   environment variables to be correctly set for dynamic loading of
   plugins. In particular, RMW_IMPLEMENTATION and AMENT_PREFIX_PATH
   need to be set, so we do this within a custom __init__.py at
   resim/ros2/__init__.py in the resultint tree.

 - We include only runfiles in resim/ros2 and resim.libs in the
   resulting tree. The external dependencies should be provided by
   other python packages we distribute, namely:
    - resim-msg
"""

load("@bazel_skylib//lib:paths.bzl", "paths")
load("@com_github_mvukov_rules_ros2//ros2:ament.bzl", "Ros2AmentSetupInfo", "ros2_ament_setup")
load("@rules_cc//cc:toolchain_utils.bzl", "find_cpp_toolchain")

RelinkedSharedObjectInfo = provider(
    fields = {
        "shared_libs": "The libraries we've re-linked",
    },
)

def _relink_dynamic_library(ctx, name, runpath, linker_inputs, **kwargs):
    """Re-link this dynamic library with the given runpath."""

    cc_toolchain = find_cpp_toolchain(ctx)
    feature_configuration = cc_common.configure_features(
        ctx = ctx,
        cc_toolchain = cc_toolchain,
        requested_features = ctx.features,
        unsupported_features = ctx.disabled_features,
    )

    linking_outputs = cc_common.link(
        actions = ctx.actions,
        feature_configuration = feature_configuration,
        cc_toolchain = cc_toolchain,
        linking_contexts = [cc_common.create_linking_context(linker_inputs = linker_inputs)],
        output_type = "dynamic_library",
        name = name,
        user_link_flags = ctx.rule.attr.linkopts + ["-Wl,-rpath={}".format(runpath)],
        **kwargs
    )
    dynamic_library = linking_outputs.library_to_link.resolved_symlink_dynamic_library

    # Do some renaming because bazel's built in link action adds lib<name>.so
    # to our shared object file and we don't have access to the private API of
    # link needed to avoid this
    output = ctx.actions.declare_file(name)
    print(name)
    ctx.actions.run(
        inputs = [dynamic_library],
        outputs = [output],
        executable = "cp",
        arguments = [dynamic_library.path, output.path],
    )

    return output

_RELINKING_PREFIX = "resim_relinking_prefix/"

def _relink_dynamic_libraries_aspect_impl(target, ctx):
    relinked_libs = []
    if hasattr(target[OutputGroupInfo], "interface_library"):
        oldlib = target[OutputGroupInfo].interface_library.to_list()[0]

        # Compute the runpath for this library.
        in_root = oldlib.short_path.startswith("resim")
        if not in_root:
            # If it's outside the root, it will be in resim.libs with all the other libraries.
            runpath = "$ORIGIN"
        else:
            # Otherwise, it will be in the root and we have to compute a relpath
            runpath_components = []
            runpath_components.append("$ORIGIN")
            runpath_components.extend([".."] * (len(oldlib.short_path.split("/")) - 1))
            runpath_components.append("resim.libs")
            runpath = "/".join(runpath_components)

        # Collect the linker inputs from the deps of this library
        linker_inputs = []
        for dep in getattr(ctx.rule.attr, "deps", []):
            linker_inputs.append(dep[CcInfo].linking_context.linker_inputs)
        linker_inputs = depset(transitive = linker_inputs)

        # Collect the compilation outputs of this rule
        compilation_outputs = cc_common.create_compilation_outputs(pic_objects = target[OutputGroupInfo].compilation_outputs)

        # Create the relinked libraries
        lib = _relink_dynamic_library(ctx, name = _RELINKING_PREFIX + oldlib.basename, compilation_outputs = compilation_outputs, runpath = runpath, linker_inputs = linker_inputs)
        relinked_libs.append(lib)

    attrs = ["srcs", "deps", "data"]
    transitive_libs = []
    for attr in attrs:
        for dep in getattr(ctx.rule.attr, attr, []):
            if RelinkedSharedObjectInfo in dep:
                transitive_libs.append(dep[RelinkedSharedObjectInfo].shared_libs)
    relinked_libs = depset(relinked_libs, transitive = transitive_libs)
    return [RelinkedSharedObjectInfo(shared_libs = relinked_libs)]

relink_dynamic_libraries_aspect = aspect(
    implementation = _relink_dynamic_libraries_aspect_impl,
    attr_aspects = ["srcs", "deps", "data"],
    attrs = {
        "_cc_toolchain": attr.label(
            default = Label("@bazel_tools//tools/cpp:current_cc_toolchain"),
        ),
    },
    toolchains = ["@bazel_tools//tools/cpp:toolchain_type"],
    fragments = ["cpp"],
)

def _ros2_py_package_rule_impl(ctx):
    outs = []
    runfiles = [dep[DefaultInfo].default_runfiles.files for dep in ctx.attr.deps]
    runfiles = depset(transitive = runfiles).to_list()
    subpackage_prefix = ctx.attr.subpackage_prefix
    for f in runfiles:
        in_root = f.short_path.startswith(subpackage_prefix)
        is_lib = f.basename.endswith(".so")
        if not in_root or is_lib:
            continue
        path = f.short_path
        out = ctx.actions.declare_file(paths.join(ctx.attr.name, path))
        ctx.actions.run(
            outputs = [out],
            inputs = [f],
            executable = "cp",
            arguments = [f.path, out.path],
        )
        outs.append(out)

    shared_libs = []
    for dep in ctx.attr.deps:
        shared_libs.append(dep[RelinkedSharedObjectInfo].shared_libs)
    shared_libs = depset(transitive = shared_libs).to_list()

    for lib in shared_libs:
        in_root = lib.short_path.startswith(subpackage_prefix)
        if in_root:
            path = lib.short_path.replace(_RELINKING_PREFIX, "")
        else:
            path = paths.join("resim.libs", lib.basename)
        out = ctx.actions.declare_file(ctx.attr.name + "/" + path)
        ctx.actions.run(
            outputs = [out],
            inputs = [lib],
            executable = "cp",
            arguments = [lib.path, out.path],
        )
        outs.append(out)

    ament = ctx.attr.ament
    prefix_path = ament[Ros2AmentSetupInfo].ament_prefix_path

    for ament_file in ctx.attr.ament[DefaultInfo].files.to_list():
        path = ament_file.short_path.replace(
            prefix_path,
            paths.join("resim.libs", "ament"),
        )
        out = ctx.actions.declare_file(paths.join(ctx.attr.name, path))
        ctx.actions.run(
            outputs = [out],
            inputs = [ament_file],
            executable = "cp",
            arguments = [ament_file.path, out.path],
        )
        outs.append(out)

    initfile = ctx.actions.declare_file(paths.join(ctx.attr.name, subpackage_prefix, "__init__.py"))
    ctx.actions.write(
        output = initfile,
        content = ctx.attr.initcontent,
    )
    outs.append(initfile)

    return [DefaultInfo(files = depset(outs))]

ros2_py_package_rule = rule(
    implementation = _ros2_py_package_rule_impl,
    attrs = {
        "ament": attr.label(providers = [Ros2AmentSetupInfo]),
        "deps": attr.label_list(aspects = [relink_dynamic_libraries_aspect]),
        "initcontent": attr.string(),
        "subpackage_prefix": attr.string(),
    },
)

def ros2_py_package(
        *,
        name,
        deps,
        subpackage_prefix,
        initcontent,
        visibility):
    ros2_py_package_rule(
        name = name,
        deps = deps,
        subpackage_prefix = subpackage_prefix,
        ament = ":{}.ament".format(name),
        initcontent = initcontent,
        visibility = visibility,
    )

    ros2_ament_setup(
        name = "{}.ament".format(name),
        deps = deps,
    )
