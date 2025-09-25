# rules_resim

This repository contains rules for creating and running ReSim builds based on oci images, and a
repository rule that makes it easy to run the CLI from bazel. Please refer to the following
documentation for details:

 - [Rule documentation](./rules.md)
 - [Repository rule documentation](./extensions.md)

Also please refer to the `examples` directory for a simple example of creating a build and a
multi-container build.

## Updating the Docs
To update the auto-generated docs with stardoc:

```bash
bazel run //:update_docs
```
