
set -e

MAIN_RELEASE_TAG=$(git ls-remote upstream refs/tags/$RESIM_VERSION)
GO_RELEASE_TAG=$(git ls-remote upstream refs/tags/go/$RESIM_VERSION)

if [[ -n "${MAIN_RELEASE_TAG}" || -n "${GO_RELEASE_TAG}" ]]; then
    echo "Tag collision! Can't release on this tag!"
    exit 1
fi
