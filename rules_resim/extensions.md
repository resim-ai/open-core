<!-- Generated with Stardoc: http://skydoc.bazel.build -->

# ReSim Extensions

<a id="resim_cli"></a>

## resim_cli

<pre>
load("@rules_resim//:extensions.bzl", "resim_cli")

resim_cli(<a href="#resim_cli-name">name</a>, <a href="#resim_cli-version">version</a>, <a href="#resim_cli-platforms">platforms</a>)
</pre>

Macro with repo rule for fetching the resim CLI.

This macro creates respos to download prebuilt `resim` CLI binaries for supported
platforms (Linux and macOS, both x86_64 and arm64). It then exposes a
`resim` target as a native binary in an interface repo, selectable by the host platform.

For example, if you use this rule in your `MODULE.bazel`:
```
resim_cli = use_extension("@rules_resim//:extensions.bzl", "resim_cli_extension")
use_repo(resim_cli, "resim_cli")
```

You can run the CLI like so.
```
bazel run @resim_cli//:resim
```


**PARAMETERS**


| Name  | Description | Default Value |
| :------------- | :------------- | :------------- |
| <a id="resim_cli-name"></a>name |  The name of the repo to create for the resim cli.   |  none |
| <a id="resim_cli-version"></a>version |  The version of the resim cli to fetch.   |  none |
| <a id="resim_cli-platforms"></a>platforms |  The platform structs containing name, platform, and sha information for this version.   |  none |


<a id="resim_cli_extension"></a>

## resim_cli_extension

<pre>
resim_cli_extension = use_extension("@rules_resim//:extensions.bzl", "resim_cli_extension")
resim_cli_extension.versions(<a href="#resim_cli_extension.versions-cli_version">cli_version</a>)
</pre>


**TAG CLASSES**

<a id="resim_cli_extension.versions"></a>

### versions

Tag class to specify the versions of fetched tools.

**Attributes**

| Name  | Description | Type | Mandatory | Default |
| :------------- | :------------- | :------------- | :------------- | :------------- |
| <a id="resim_cli_extension.versions-cli_version"></a>cli_version |  The CLI release version (e.g. v0.29.0)   | String | optional |  `""`  |


