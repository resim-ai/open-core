<!-- Generated with Stardoc: http://skydoc.bazel.build -->

# ReSim Extensions

<a id="resim_cli_extension"></a>

## resim_cli_extension

<pre>
resim_cli_extension = use_extension("@rules_resim//:extensions.bzl", "resim_cli_extension")
resim_cli_extension.versions(<a href="#resim_cli_extension.versions-cli_version">cli_version</a>)
</pre>

Extension for pulling in the resim cli for multiple platforms.

For example, if you use this rule in your `MODULE.bazel`:
```
resim_cli = use_extension("@rules_resim//:extensions.bzl", "resim_cli_extension")
use_repo(resim_cli, "resim_cli")
```
You can run the CLI like so.
```
bazel run @resim_cli//:resim
```


**TAG CLASSES**

<a id="resim_cli_extension.versions"></a>

### versions

Tag class to specify the versions of fetched tools.

**Attributes**

| Name  | Description | Type | Mandatory | Default |
| :------------- | :------------- | :------------- | :------------- | :------------- |
| <a id="resim_cli_extension.versions-cli_version"></a>cli_version |  The CLI release version (e.g. v0.29.0)   | String | optional |  `""`  |


