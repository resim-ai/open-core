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
        stubgen = "//bazel:stubgen",
        testonly = kwargs.get("testonly"),
    )

    native.filegroup(
        name = name + "-stubs",
        srcs = [name + ".pyi"],
        visibility = kwargs.get("visibility"),
        testonly = kwargs.get("testonly"),
    )

stubgen_template = """
cd $BUILD_WORKSPACE_DIRECTORY
export PYTHONPATH={python_path}
{stubgen_binary} {args}
"""

def _stubgen_impl(ctx):
    # Get the binary and input file
    stubgen_binary = ctx.executable.stubgen

    input = ctx.attr.dep.files.to_list()[0]

    package_path = input.short_path.removesuffix(input.basename).replace("/", ".")
    module_name = package_path + input.basename.split(".")[0]

    args = []
    for f in ctx.attr.dep.default_runfiles.files.to_list():
        if f.basename.endswith(".so"):
            # Should remove suffix rather than strip
            args.extend(["-m", f.short_path.removesuffix(f.basename).replace("/", ".") + f.basename.split(".")[0]])

    args.extend(["-m", module_name])
    args.extend(["-o", "."])

    script = ctx.actions.declare_file(ctx.label.name)
    script_content = stubgen_template.format(
        python_path = ctx.genfiles_dir.path,
        stubgen_binary = stubgen_binary.path,
        args = " ".join(args),
    )
    ctx.actions.write(script, script_content, is_executable = True)

    return DefaultInfo(
        executable = script,
        runfiles = ctx.runfiles(files = [input] + ctx.attr.dep.default_runfiles.files.to_list()),
    )

stubgen = rule(
    implementation = _stubgen_impl,
    executable = True,
    attrs = {
        "dep": attr.label(allow_single_file = True),  # Input shared library
        "stubgen": attr.label(
            executable = True,
            doc = "Path to the pybind11-stubgen binary",
            cfg = "exec",
        ),
    },
)
