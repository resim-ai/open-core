#!/usr/bin/env bash
set -euo pipefail

if [[ -n "${STAMP_FILE_PATH:-}" ]]; then
    if [[ -z "${RESIM_VERSION:-}" ]]; then
        STAMP_VERSION=$(grep "^STABLE_RESIM_VERSION " "${STAMP_FILE_PATH}" | cut -d' ' -f2- | xargs)
        if [[ -n "${STAMP_VERSION:-}" ]]; then
            RESIM_VERSION="${STAMP_VERSION}"
        fi
    fi
    
    if [[ -z "${RESIM_BRANCH:-}" ]]; then
        STAMP_BRANCH=$(grep "^STABLE_RESIM_BRANCH " "${STAMP_FILE_PATH}" | cut -d' ' -f2- | xargs)
        if [[ -n "${STAMP_BRANCH:-}" ]]; then
            RESIM_BRANCH="${STAMP_BRANCH}"
        fi
    fi
fi

if [[ -z "${RESIM_VERSION:-}" ]]; then
    echo "Error: RESIM_VERSION is not set" >&2
    exit 1
fi

if [[ -z "${RESIM_BRANCH:-}" ]]; then
    echo "Error: RESIM_BRANCH is not set" >&2
    exit 1
fi

echo "${RESIM_VERSION}" > "${VERSION_FILE_PATH}"
echo "${RESIM_BRANCH}" > "${BRANCH_FILE_PATH}"
