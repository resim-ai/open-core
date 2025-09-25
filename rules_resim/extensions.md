<!-- Generated with Stardoc: http://skydoc.bazel.build -->

# ReSim Extensions

<a id="resim_cli"></a>

## resim_cli

<pre>
load("@rules_resim//:extensions.bzl", "resim_cli")

resim_cli(<a href="#resim_cli-name">name</a>, <a href="#resim_cli-repo_mapping">repo_mapping</a>)
</pre>

Repo rule for fetching the resim CLI.

This repository rule downloads prebuilt `resim` CLI binaries for supported
platforms (Linux and macOS, both x86_64 and arm64). It then exposes a
`resim` target as a native binary, selectable by the host platform.

For example, if you use this rule in your `MODULE.bazel`:
```
resim_cli = use_extension("@rules_resim//:extensions.bzl", "resim_cli_extension")
use_repo(resim_cli, "resim_cli")
```

You can run the CLI like so.
```
bazel run @resim_cli//:resim
```

**ATTRIBUTES**


| Name  | Description | Type | Mandatory | Default |
| :------------- | :------------- | :------------- | :------------- | :------------- |
| <a id="resim_cli-name"></a>name |  A unique name for this repository.   | <a href="https://bazel.build/concepts/labels#target-names">Name</a> | required |  |
| <a id="resim_cli-repo_mapping"></a>repo_mapping |  In `WORKSPACE` context only: a dictionary from local repository name to global repository name. This allows controls over workspace dependency resolution for dependencies of this repository.<br><br>For example, an entry `"@foo": "@bar"` declares that, for any time this repository depends on `@foo` (such as a dependency on `@foo//some:target`, it should actually resolve that dependency within globally-declared `@bar` (`@bar//some:target`).<br><br>This attribute is _not_ supported in `MODULE.bazel` context (when invoking a repository rule inside a module extension's implementation function).   | <a href="https://bazel.build/rules/lib/dict">Dictionary: String -> String</a> | optional |  |


<a id="resim_cli_extension"></a>

## resim_cli_extension

<pre>
resim_cli_extension = use_extension("@rules_resim//:extensions.bzl", "resim_cli_extension")
</pre>



