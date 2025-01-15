# Copyright 2025 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

load("@pybind11_bazel//:build_defs.bzl", _pybind_extension = "pybind_extension")

def pybind_extension(
        name,
        srcs,
        visibility,
        deps):
    _pybind_extension(
        name = name,
        srcs = srcs,
        visibility = visibility,
        deps = deps,
    )

    stubgen(
        name = name + ".stubs",
        dep = ":{}".format(name),
        stubgen = "//bazel:stubgen",
    )

def _stubgen_impl(ctx):
    # Get the binary and input file
    stubgen_binary = ctx.executable.stubgen

    output = ctx.actions.declare_directory(ctx.attr.name)
    input = ctx.attr.dep.files.to_list()[0]

    package_path = input.short_path.removesuffix(input.basename).replace("/", ".")
    module_name = package_path + input.basename.split(".")[0]

    args = ctx.actions.args()
    for f in ctx.attr.dep.default_runfiles.files.to_list():
        if f.basename.endswith(".so"):
            # Should remove suffix rather than strip
            args.add("-m", f.short_path.removesuffix(f.basename).replace("/", ".") + f.basename.split(".")[0])

    args.add("-m", module_name)
    args.add("-o", output.path)
    ctx.actions.run(
        executable = stubgen_binary,
        tools = [stubgen_binary],
        inputs = [input] + ctx.attr.dep.default_runfiles.files.to_list(),
        outputs = [output],
        arguments = [args],
        env = {"PYTHONPATH": ctx.genfiles_dir.path},
    )
    return DefaultInfo(
        files = depset([output]),
    )

stubgen = rule(
    implementation = _stubgen_impl,
    attrs = {
        "dep": attr.label(allow_single_file = True),  # Input shared library
        "stubgen": attr.label(
            executable = True,
            doc = "Path to the pybind11-stubgen binary",
            cfg = "exec",
        ),
    },
)
