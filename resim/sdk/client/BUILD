load("@rules_python//python/entry_points:py_console_script_binary.bzl", "py_console_script_binary")

py_console_script_binary(
    name = "openapi-python-client",
    pkg = "@resim_python_client_dev_deps//openapi_python_client",
)

exports_files([
    "requirements.txt",
    "openapi_python_client_config.yaml",
    "openapi.yaml",
])
