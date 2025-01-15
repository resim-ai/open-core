"""Python & PyBind rule customizations."""
# Copyright 2025 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

load("@pybind11_bazel//:build_defs.bzl", _pybind_extension = "pybind_extension")

def pybind_extension(
        name,
        **kwargs):
    _pybind_extension(
        name = name,
        **kwargs
    )

    stubgen(
        name = name + ".stubgen",
        dep = ":{}".format(name),
        stubgen = "//bazel:pybind11-stubgen",
        testonly = kwargs.get("testonly"),
        numpy = "@@rules_python~~pip~resim_python_deps_310_numpy//:pkg",
    )

    # LicenseInfo, PyInfo, PyInfo, InstrumentedFilesInfo, PyCcLinkParamsProvider, OutputGroupInfo]
    native.filegroup(
        name = name + "-stubs",
        srcs = [name + ".pyi"],
        visibility = kwargs.get("visibility"),
        testonly = kwargs.get("testonly"),
    )

stubgen_template = """
pwd
export PYTHONPATH="$BUILD_WORKSPACE_DIRECTORY:{python_path}"
echo "$PYTHONPATH"
{stubgen_binary} {args} -o $BUILD_WORKSPACE_DIRECTORY
"""

def _stubgen_impl(ctx):
    # Get the binary and input file
    stubgen_binary = ctx.executable.stubgen

    input = ctx.attr.dep.files.to_list()[0]

    package_path = input.short_path.removesuffix(input.basename).replace("/", ".")
    module_name = package_path + input.basename.split(".")[0]

    args = []
    args.extend([module_name])
    args.extend(["-o", "."])

    script = ctx.actions.declare_file(ctx.label.name)
    script_content = stubgen_template.format(
        python_path = ";".join(["./external/{}".format(imp) for imp in ctx.attr.numpy[PyInfo].imports.to_list()]),
        stubgen_binary = stubgen_binary.short_path,
        args = " ".join(args),
    )

    ctx.actions.write(script, script_content, is_executable = True)

    return DefaultInfo(
        executable = script,
        runfiles = ctx.runfiles(files = [input] + ctx.attr.numpy.default_runfiles.files.to_list() + ctx.attr.dep.default_runfiles.files.to_list() +
                                        ctx.attr.stubgen.default_runfiles.files.to_list()),
    )

stubgen = rule(
    implementation = _stubgen_impl,
    executable = True,
    attrs = {
        "dep": attr.label(allow_single_file = True),  # Input shared library
        "numpy": attr.label(),
        "stubgen": attr.label(
            executable = True,
            doc = "Path to the pybind11-stubgen binary",
            cfg = "target",
        ),
    },
)
