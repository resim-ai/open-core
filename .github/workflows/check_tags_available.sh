#!/bin/bash

set -e

MAIN_RELEASE_TAG=$(git ls-remote origin refs/tags/$RESIM_VERSION)
GO_RELEASE_TAG=$(git ls-remote origin refs/tags/$RESIM_VERSION-go)

if [[ -n "${MAIN_RELEASE_TAG}" || -n "${GO_RELEASE_TAG}" ]]; then
    echo "Tag collision! Can't release on this tag!"
    exit 1
fi
