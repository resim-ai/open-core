

ImagePushInfo = provider(
    fields = {
        "repository": "The image repository name (string)",
        "remote_tags": "The image remote tags (list of strings)",
    }
)

def _print_image_aspect_impl(target, ctx):
    repo = getattr(ctx.rule.attr, "repository", None)
    tags = getattr(ctx.rule.attr, "remote_tags", [])
    return [ImagePushInfo(
        repository = repo,
        remote_tags = tags,
    )]

print_image_aspect = aspect(
    implementation = _print_image_aspect_impl,
    attr_aspects = ["image_pushes"],
)


def _get_image_uri(image_push):
    if ImagePushInfo not in image_push:
        fail("Could not determine push info for target: %s" % image_push.label)

    info = image_push[ImagePushInfo]

    tag_file_list = info.remote_tags.files.to_list()
    if len(tag_file_list) != 1:
        fail("Could not determine push info for target: %s" % image_push.label)

    return info.repository, tag_file_list[0]

def _resim_build_impl(ctx):
    
    tagfile_path = ""
    if len(ctx.attr.image_pushes) == 1 and ctx.attr.build_spec == None:
        image, tagfile = _get_image_uri(ctx.attr.image_pushes[0])
        tagfile_path = tagfile.short_path

    out = ctx.actions.declare_file(ctx.label.name + ".sh")
    runfiles = ctx.runfiles(files = [out, ctx.executable._resim_cli])
    
    for image_push in ctx.attr.image_pushes:
        info = image_push[DefaultInfo]
        if hasattr(info, "default_runfiles"):
            runfiles = runfiles.merge(info.default_runfiles)
        if hasattr(info, "runfiles"):
            runfiles = runfiles.merge(info.runfiles)

    ctx.actions.expand_template(
        output = out,
        template = ctx.file._wrapper_template,
        substitutions = {
            "%{RESIM_CLI}": ctx.executable._resim_cli.short_path,
            "%{TAGFILE_PATH}": tagfile_path
        },
        is_executable = True,
    )
    return [DefaultInfo(files = depset([out]), runfiles=runfiles, executable=out)]
    
resim_build = rule(
    implementation = _resim_build_impl,
    attrs = {
        "project": attr.string(
            mandatory = True,
            doc = "The name or ID of the project to create the build in"
        ),
        "system": attr.string(
            mandatory = True,
            doc = "The name or ID of the system the build is an instance of"
        ),
        "branch": attr.string(
            mandatory = True,
            doc = "The name or ID of the branch to nest the build in, usually the associated git branch"
        ),
        "version": attr.string(
            mandatory = True,
            doc = "The version of the build image, usually a commit ID"
        ),
        "resim_name": attr.string(
            doc = "The name of the build"
        ),
        "description": attr.string(
            mandatory = True,
            doc = "The description of the build, often a commit message (can include markdown). For backwards compatibility reasons, if name is omitted, the description will be used"
        ),
        "auto_create_branch": attr.bool(
            default = True,
            doc = "Whether to automatically create branch if it doesn't exist"
        ),
        "build_spec": attr.label(
            allow_single_file = True,
            doc = "Paths to main compose file (may use extends)"
        ),
        "image_pushes": attr.label_list(
            allow_files = False,
            doc = "List of oci image pushes",
            aspects = [print_image_aspect],
        ),
        "build_spec_env": attr.string_dict(
            doc = "Environment variables to set for the build_spec", 
        ),
        "_wrapper_template": attr.label(
            allow_single_file = True,
            default = ":wrapper.sh.tpl",
        ),
        "_resim_cli": attr.label(
            allow_single_file = True,
            executable = True,
            default = "@resim_cli//:resim",
            cfg = "target",
        ),
    },
    executable = True,
)
