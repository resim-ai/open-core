"""
This file contains a single rule which uses the openapi-python-client package to generate an openapi pythonclient based on a given api specification and generator config.
"""

def _openapi_client_impl(ctx):
    """Generate an openapi client without the ruff post hook"""
    output_dir = ctx.actions.declare_directory(ctx.attr.package_name)

    # Filter the specification file to remove lines that cause issues with the
    # generator.
    specification_file = ctx.attr.specification.files.to_list()[0]
    filtered_specification = ctx.actions.declare_file(specification_file.basename.replace(".yaml", ".filtered.yaml"))

    ctx.actions.run_shell(
        inputs = [specification_file],
        outputs = [filtered_specification],
        command = "cat {src} | sed '/format: uuid/s/^/#/g' > {out}".format(
            src = specification_file.path,
            out = filtered_specification.path,
        ),
    )

    args = ctx.actions.args()
    args.add("generate")

    args.add("--path")
    specification = filtered_specification.path
    args.add(specification)

    args.add("--config")
    config = ctx.attr.config.files.to_list()[0]
    args.add(config)

    args.add("--output-path")
    args.add(output_dir.path)

    args.add("--meta")
    args.add("none")

    args.add("--fail-on-warning")

    args.add("--overwrite")

    generator = ctx.executable._generator

    ctx.actions.run(
        use_default_shell_env = True,
        mnemonic = "GeneratePythonClient",
        executable = generator,
        arguments = [args],
        inputs = depset([filtered_specification], transitive = [ctx.attr.config.files]),
        outputs = [output_dir],
    )

    return [DefaultInfo(files = depset([output_dir]))]

openapi_client = rule(
    implementation = _openapi_client_impl,
    attrs = {
        "config": attr.label(mandatory = True, allow_single_file = [".yml", ".yaml"]),
        "package_name": attr.string(mandatory = True),
        "specification": attr.label(mandatory = True, allow_single_file = [".yml", ".yaml"]),
        "_generator": attr.label(
            default = Label(":openapi-python-client"),
            executable = True,
            cfg = "exec",
        ),
    },
)
