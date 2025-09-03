


def _resim_build_impl(ctx):
    pass



resim_build = rule(
    implementation = _resim_build_impl,
    attrs = {
        # Required flags
        "branch": attr.string(
            mandatory = True,
            doc = "The name or ID of the branch to nest the build in, usually the associated git branch"
        ),
        "description": attr.string(
            mandatory = True,
            doc = "The description of the build, often a commit message (can include markdown). For backwards compatibility reasons, if name is omitted, the description will be used"
        ),
        "project": attr.string(
            mandatory = True,
            doc = "The name or ID of the project to create the build in"
        ),
        "system": attr.string(
            mandatory = True,
            doc = "The name or ID of the system the build is an instance of"
        ),
        "version": attr.string(
            mandatory = True,
            doc = "The version of the build image, usually a commit ID"
        ),
        # Optional flags
        "auto_create_branch": attr.bool(
            default = True,
            doc = "Whether to automatically create branch if it doesn't exist"
        ),
        "build_spec": attr.label(
            allow_single_file = True,
            doc = "Paths to main compose file (may use extends)"
        ),
        "compose_profiles": attr.string_list(
            doc = "Profiles to use when parsing the build spec"
        ),
        "env_files": attr.label_list(
            allow_files = True,
            doc = "Paths to env files to use when parsing the build spec"
        ),
        "github": attr.bool(
            default = False,
            doc = "Whether to output format in github action friendly format"
        ),
        "image_pushes": attr.label_list(
            allow_files = False,
            executable = True,
            doc = "List of executable target labels"
        ),
        "resim_name": attr.string(
            doc = "The name of the build"
        ),
        "show_build_spec_only": attr.bool(
            default = False,
            doc = "Print the build spec only, formatted as YAML"
        ),
        "build_spec_env": attr.string_dict(
            doc = "Environment variables to set for the build_spec", 
        ),
    },
)
