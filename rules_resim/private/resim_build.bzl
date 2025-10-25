# Copyright 2025 ReSim, Inc.
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""A rule for creating resim builds."""

load("@resim_cli//:defs.bzl", "RESIM_CLI_VERSION")
load(":cli.bzl", "version_to_tuple")

ImagePushInfo = provider(
    "A provider to help us get repo information from oci_push() targets",
    fields = {
        "remote_tags": "The image remote tags (list of strings)",
        "repository": "The image repository name (string)",
    },
)

ReSimBuildInfo = provider(
    doc = "Carries project information from build rules to other rules",
    fields = {
        "project": "The name or ID of the project for this build",
    },
)

def _get_image_uri_aspect_impl(_target, ctx):
    repo = getattr(ctx.rule.attr, "repository", None)
    tags = getattr(ctx.rule.attr, "remote_tags", [])
    return [ImagePushInfo(
        repository = repo,
        remote_tags = tags,
    )]

get_image_uri_aspect = aspect(
    implementation = _get_image_uri_aspect_impl,
)

def _get_image_uri(image_push):
    if ImagePushInfo not in image_push:
        fail("Could not determine push info for target: %s" % image_push.label)

    info = image_push[ImagePushInfo]

    tag_file_list = info.remote_tags.files.to_list()
    if len(tag_file_list) == 0:
        fail("Could not determine push info for target: %s" % image_push.label)

    return info.repository, tag_file_list[0]

def _resim_build_impl(ctx):
    _, minor, _ = version_to_tuple(RESIM_CLI_VERSION)
    if minor < 19:
        fail("Unsupported CLI version for rule: {}".format(RESIM_CLI_VERSION))

    out = ctx.actions.declare_file(ctx.label.name + ".sh")
    runfiles = ctx.runfiles(files = [out, ctx.executable._resim_cli])
    runfiles = runfiles.merge(ctx.runfiles(transitive_files = depset(ctx.files.data)))

    push_cmds = []
    for image_push in ctx.attr.image_pushes:
        info = image_push[DefaultInfo]
        if hasattr(info, "default_runfiles"):
            runfiles = runfiles.merge(info.default_runfiles)
        if hasattr(info, "runfiles"):
            runfiles = runfiles.merge(info.runfiles)

        push_cmds.append("./" + image_push[DefaultInfo].files_to_run.executable.short_path)

    if ctx.attr.build_spec == None:
        if len(ctx.attr.image_pushes) > 1:
            fail("More than one image push provided for non mcb")

        repository, tagfile = _get_image_uri(ctx.attr.image_pushes[0])
        tagfile_path = tagfile.short_path
        ctx.actions.expand_template(
            output = out,
            template = ctx.file._wrapper_template,
            substitutions = {
                "%{AUTO_CREATE_BRANCH}": str(ctx.attr.auto_create_branch).lower(),
                "%{BRANCH}": ctx.attr.branch,
                "%{DESCRIPTION}": ctx.attr.description,
                "%{PROJECT}": ctx.attr.project,
                "%{PUSH_CMDS}": "\n".join(push_cmds),
                "%{REPOSITORY}": repository,
                "%{RESIM_CLI}": ctx.executable._resim_cli.short_path,
                "%{RESIM_NAME}": ctx.attr.resim_name if ctx.attr.resim_name else ctx.attr.name,
                "%{SYSTEM}": ctx.attr.system,
                "%{TAGFILE_PATH}": tagfile_path,
                "%{VERSION}": ctx.attr.version,
            },
            is_executable = True,
        )
    else:
        env_file = ctx.actions.declare_file(ctx.label.name + ".env")
        env_lines = ["%s=%s" % (k, v) for k, v in ctx.attr.build_spec_env.items()]
        ctx.actions.write(
            output = env_file,
            content = "\n".join(env_lines) + "\n",
        )

        runfiles = runfiles.merge(ctx.runfiles(files = [ctx.file.build_spec, env_file]))

        ctx.actions.expand_template(
            output = out,
            template = ctx.file._wrapper_mcb_template,
            substitutions = {
                "%{AUTO_CREATE_BRANCH}": str(ctx.attr.auto_create_branch).lower(),
                "%{BRANCH}": ctx.attr.branch,
                "%{BUILD_SPEC}": ctx.file.build_spec.short_path,
                "%{DESCRIPTION}": ctx.attr.description,
                "%{ENV_FILE_PATH}": env_file.short_path,
                "%{PROJECT}": ctx.attr.project,
                "%{PUSH_CMDS}": "\n".join(push_cmds),
                "%{RESIM_CLI}": ctx.executable._resim_cli.short_path,
                "%{RESIM_NAME}": ctx.attr.resim_name if ctx.attr.resim_name else ctx.attr.name,
                "%{SYSTEM}": ctx.attr.system,
                "%{VERSION}": ctx.attr.version,
            },
            is_executable = True,
        )
    return [DefaultInfo(files = depset([out]), runfiles = runfiles, executable = out), ReSimBuildInfo(project = ctx.attr.project)]

resim_build = rule(
    implementation = _resim_build_impl,
    attrs = {
        "auto_create_branch": attr.bool(
            default = True,
            doc = "Whether to automatically create branch if it doesn't exist",
        ),
        "branch": attr.string(
            mandatory = True,
            doc = "The name or ID of the branch to nest the build in, usually the associated git branch",
        ),
        "build_spec": attr.label(
            allow_single_file = True,
            doc = "Paths to main compose file (may use extends)",
        ),
        "build_spec_env": attr.string_dict(
            doc = "Environment variables to set for the build_spec",
        ),
        "data": attr.label_list(
            allow_files = True,
            doc = "List of data runfiles",
        ),
        "description": attr.string(
            mandatory = True,
            doc = "The description of the build, often a commit message (can include markdown). For backwards compatibility reasons, if name is omitted, the description will be used",
        ),
        "image_pushes": attr.label_list(
            allow_files = False,
            doc = "List of oci image pushes",
            aspects = [get_image_uri_aspect],
        ),
        "project": attr.string(
            mandatory = True,
            doc = "The name or ID of the project to create the build in",
        ),
        "resim_name": attr.string(
            doc = "The name of the build",
        ),
        "system": attr.string(
            mandatory = True,
            doc = "The name or ID of the system the build is an instance of",
        ),
        "version": attr.string(
            mandatory = True,
            doc = "The version of the build image, usually a commit ID",
        ),
        "_resim_cli": attr.label(
            allow_single_file = True,
            executable = True,
            default = "@resim_cli//:resim",
            cfg = "target",
        ),
        "_wrapper_mcb_template": attr.label(
            allow_single_file = True,
            default = ":wrapper_mcb.sh.tpl",
        ),
        "_wrapper_template": attr.label(
            allow_single_file = True,
            default = ":wrapper.sh.tpl",
        ),
    },
    executable = True,
    doc = """A rule for creating ReSim builds easily

This rule is designed to facilitate the creation of ReSim single-container and multi-container resim builds.""",
)
