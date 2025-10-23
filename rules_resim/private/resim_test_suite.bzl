# Copyright 2025 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""A rule for running resim test suites."""

load("@resim_cli//:defs.bzl", "RESIM_CLI_VERSION")
load(":cli.bzl", "version_to_tuple")
load(":resim_build.bzl", "ReSimBuildInfo")

def _resim_test_suite_run_impl(ctx):
    _, minor, _ = version_to_tuple(RESIM_CLI_VERSION)
    if minor < 19:
        fail("Unsupported CLI version for rule: {}".format(RESIM_CLI_VERSION))

    out = ctx.actions.declare_file(ctx.label.name + ".sh")
    runfiles = ctx.runfiles(files = [out, ctx.executable._resim_cli])
    additional_flags = []

    if ctx.attr.allowable_failure_percent:
        additional_flags.extend(["--allowable-failure-percent", str(ctx.attr.allowable_failure_percent)])
    if ctx.attr.pool_labels:
        additional_flags.extend(["--pool-labels", "\\\"{}\\\"".format(",".join(ctx.attr.pool_labels))])

    create_build_info = ctx.attr.resim_build[DefaultInfo]
    if hasattr(create_build_info, "default_runfiles"):
        runfiles = runfiles.merge(create_build_info.default_runfiles)
    if hasattr(create_build_info, "runfiles"):
        runfiles = runfiles.merge(create_build_info.runfiles)
    create_build_cmd = "./" + create_build_info.files_to_run.executable.short_path

    ctx.actions.expand_template(
        output = out,
        template = ctx.file._wrapper_template,
        substitutions = {
            "%{ADDITIONAL_FLAGS}": " ".join(additional_flags),
            "%{CREATE_BUILD_CMD}": create_build_cmd,
            "%{PROJECT}": ctx.attr.resim_build[ReSimBuildInfo].project,
            "%{RESIM_CLI}": ctx.executable._resim_cli.short_path,
            "%{TEST_SUITE}": ctx.attr.test_suite,
        },
        is_executable = True,
    )
    return [DefaultInfo(files = depset([out]), runfiles = runfiles, executable = out)]

resim_test_suite_run = rule(
    implementation = _resim_test_suite_run_impl,
    attrs = {
        "allowable_failure_percent": attr.int(
            doc = "An optional percentage (0-100) that determines the maximum percentage of tests that can have an execution error and have aggregate metrics be computed and consider the batch successfully completed. If not supplied, ReSim defaults to 0, which means that the batch will only be considered successful if all tests complete successfully.",
        ),
        "pool_labels": attr.string_list(
            doc = "Pool labels to determine where to run this test suite. Pool labels are interpreted as a logical AND.",
        ),
        "resim_build": attr.label(
            mandatory = True,
            allow_files = False,
            providers = [ReSimBuildInfo],
            doc = "A resim build rule to run a test suite on.",
        ),
        "test_suite": attr.string(
            mandatory = True,
            doc = "The name or ID of the test suite to run.",
        ),
        "_resim_cli": attr.label(
            allow_single_file = True,
            executable = True,
            default = "@resim_cli//:resim",
            cfg = "target",
        ),
        "_wrapper_template": attr.label(
            allow_single_file = True,
            default = ":test_suite_wrapper.sh.tpl",
        ),
    },
    executable = True,
)
