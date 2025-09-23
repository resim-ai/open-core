"""Extension for the resim CLI."""

load("@rules_resim//private:cli.bzl", "resim_cli")

def _extension_impl(_):
    resim_cli(name = "resim_cli")

resim_cli_extension = module_extension(
    implementation = _extension_impl,
)
