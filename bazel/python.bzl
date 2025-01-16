# Copyright 2025 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Python & PyBind rule customizations."""

load("@pybind11_bazel//:build_defs.bzl", _pybind_extension = "pybind_extension")

def pybind_extension(
        name,
        **kwargs):
    _pybind_extension(
        name = name,
        **kwargs
    )

    stubgen(
        name = name + ".stubs",
        extension = ":{}".format(name),
        testonly = kwargs.get("testonly"),
        visibility = kwargs.get("visibility"),
    )

def _get_module_name(pybind_binary_path):
    """ Simple helper to get a module path from a binary path.

        For example, when given "resim/transforms/python/se3_python.so", this will return
        "resim.transforms.python.se3_python"
    """
    package_path = pybind_binary_path.short_path.removesuffix(pybind_binary_path.basename).replace("/", ".")
    return package_path + pybind_binary_path.basename.split(".")[0]

def _stubgen_impl(ctx):
    """Implementation of the stubgen rule which generates stubs (e.g. *.pyi files) from shared object files."""
    stubgen_binary = ctx.executable._stubgen

    args = ctx.actions.args()

    stubs_dir = ctx.actions.declare_directory(ctx.attr.name)
    args.add("-o", stubs_dir.path)

    inputs = ctx.attr.extension.files.to_list()
    if len(inputs) != 1:
        fail("Wrong number of inputs!")
    input = inputs[0]

    args.add("-m", _get_module_name(input))
    ctx.actions.run(
        mnemonic = "GenerateStubs",
        inputs = inputs + ctx.attr.extension.default_runfiles.files.to_list(),
        outputs = [stubs_dir],
        executable = stubgen_binary,
        arguments = [args],
        env = {"PYTHONPATH": ctx.genfiles_dir.path},
    )

    # Move the stubs file out to the bazel hierarchy as well so we can package
    # it more easily in python wheels.
    stubs_path = stubs_dir.path + "/" + input.short_path.removesuffix(".so") + ".pyi"
    stubs_file = ctx.actions.declare_file(input.basename.removesuffix(".so") + ".pyi")
    ctx.actions.run_shell(
        command = "cp {} {}".format(stubs_path, stubs_file.path),
        inputs = [stubs_dir],
        outputs = [stubs_file],
    )
    return DefaultInfo(files = depset([stubs_file]))

stubgen = rule(
    doc = """
    This rule is designed to generate a stubs file
    (https://mypy.readthedocs.io/en/stable/stubs.html) using mypy.

    Users pass in a binary extension (e.g. quaternion.so) generated by pybind11, and a stubs file
    (e.g. quaternion.pyi) is generated as an output.
    """,
    implementation = _stubgen_impl,
    attrs = {
        "extension": attr.label(
            allow_single_file = True,
            doc = "The extension file (e.g. quaternion.so) to generate stubs for.",
        ),
        "_stubgen": attr.label(
            executable = True,
            doc = "Path to the stubgen binary",
            cfg = "exec",
            default = ":stubgen",
        ),
    },
)
