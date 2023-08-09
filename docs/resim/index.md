# ReSim Open Libraries

ReSim's `open-core` repository contains the open source subset of ReSim's C++
code intended to accelerate robotics development.

## Getting Started

### Install and Setup Docker

These instructions vary by host platform: 

  - [Mac](https://docs.docker.com/desktop/mac/install/)
  - [Windows](https://docs.docker.com/desktop/windows/install/)
  - [Linux](https://docs.docker.com/engine/install/ubuntu/)
    - [post-install](https://docs.docker.com/engine/install/linux-postinstall/)

### Get the Code

```
git clone git@github.com:resim-ai/open-core.git
```

### Setting up the Environment

In order to easily build and interact with ReSim's code, you need our
development docker image. To fetch this image, navigate to the open source repo
and call the following script.

```bash
cd open-core
./.devcontainer/pull.sh
```

If you have any trouble pulling here, please see
[this section](#building-the-docker-image-locally) below which
shows how to build your own image locally. Next, we can start up a development
docker container locally:

```bash
./.devcontainer/run.sh
```

### Building the Code

We use [Bazel](https://bazel.build/) as our build system. Therefore, to build
all of the example binaries you simply have to execute:

```bash
bazel build //resim/examples/...
```

Now, all of the binaries will be present in `bazel-bin/resim/examples/`.
Individual binaries can be built and run in a single step using `bazel run`.
For example:

```bash
bazel run //resim/examples:liegroups
```

### Autocomplete via `clangd`, `clang-tidy`, and `compile_commands.json`
Many developer tools (such as `clangd` and `clang-tidy`) require a `compile_commands.json` file in the root of the tree.  This file is large and prone to change,
so we don't check it in and each developer is responsible for generating it themselves.  You will need to regenerate it whenever you change your BUILD files.

We use [Hedron's Compile Commands Extractor](https://github.com/hedronvision/bazel-compile-commands-extractor) to generate `compile_commands.json`.  It can be invoked with:

```bash
bazel run @hedron_compile_commands//:refresh_all
```

### Using Dotfiles

If you'd like to use your own dotfiles for customization inside the container,
our recommendation is to create a dotfiles repository and write an `install.sh`
script to automatically move those dotfiles to the appropriate location (and
perform whatever other workspace customization you'd like).  Note: do not put
credentials of any kind in this repository.

If you'd like to streamline this system a bit, you can check out your dotfiles
repo to `$HOME/dotfiles` and use `.devcontainer/run_dotfiles.sh` (instead of
`.devcontainer/run.sh`) to have it automatically mounted and installed in your
container.  

### Building the Docker Image Locally

We also provide scripts to build the development Docker image locally. To do
this, simply do:

```bash
./.devcontainer/build.sh
```
And then once this is complete simply use `.devcontainer/run_local.sh` or
`.devcontainer/run_dotfiles_local.sh` instead of `.devcontainer/run.sh` or
`.devcontainer/run_dotfiles.sh`.
