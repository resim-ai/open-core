"""
This file contains a single rule which uses the openapi-python-client package to generate an openapi pythonclient based on a given api specification and generator config.
"""

def _openapi_client_impl(ctx):
    """Generate an openapi client without the ruff post hook"""
    output_dir = ctx.actions.declare_directory(ctx.attr.name)

    args = ctx.actions.args()
    args.add("generate")

    args.add("--path")
    specification = ctx.attr.specification.files.to_list()[0].path
    args.add(specification)

    args.add("--config")
    config = ctx.attr.config.files.to_list()[0]
    args.add(config)

    args.add("--output-path")
    args.add(output_dir.path)

    args.add("--overwrite")

    generator = ctx.executable._generator

    ctx.actions.run(
        use_default_shell_env = True,
        mnemonic = "GeneratePythonClient",
        executable = generator,
        arguments = [args],
        inputs = depset([], transitive = [ctx.attr.specification.files, ctx.attr.config.files]),
        outputs = [output_dir],
    )

    return [DefaultInfo(files = depset([output_dir]))]

openapi_client = rule(
    implementation = _openapi_client_impl,
    attrs = {
        "config": attr.label(mandatory = True, allow_single_file = [".yml", ".yaml"]),
        "specification": attr.label(mandatory = True, allow_single_file = [".yml", ".yaml"]),
        "_generator": attr.label(
            default = Label(":openapi-python-client"),
            executable = True,
            cfg = "exec",
        ),
    },
)
