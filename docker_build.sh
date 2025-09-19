#!/bin/bash

set -euo pipefail

#
# Build Docker image for erp42-ros:humble
# - Uses BuildKit/Buildx when available for caching and performance.
# - Respects optional cache env vars:
#     BUILDX_CACHE_FROM (e.g., "type=local,src=/tmp/.buildx-cache")
#     BUILDX_CACHE_TO   (e.g., "type=local,dest=/tmp/.buildx-cache-new,mode=max")
#

IMAGE_TAG=${IMAGE_TAG:-erp42-ros:humble}
DOCKERFILE_PATH=${DOCKERFILE_PATH:-docker/Dockerfile}
BUILD_CONTEXT=${BUILD_CONTEXT:-.}

UID_ARG=${UID:-$(id -u)}
GID_ARG=${GID:-$(id -g)}

export DOCKER_BUILDKIT=${DOCKER_BUILDKIT:-1}

if docker buildx version >/dev/null 2>&1; then
  # Use buildx with optional cache and load to local docker
  CMD=(docker buildx build
       --network host
       -f "${DOCKERFILE_PATH}"
       --build-arg "UID=${UID_ARG}"
       --build-arg "GID=${GID_ARG}"
       -t "${IMAGE_TAG}"
       --load
  )

  # Cache from/to if provided
  if [[ -n "${BUILDX_CACHE_FROM:-}" ]]; then
    CMD+=(--cache-from "${BUILDX_CACHE_FROM}")
  fi
  if [[ -n "${BUILDX_CACHE_TO:-}" ]]; then
    CMD+=(--cache-to "${BUILDX_CACHE_TO}")
  fi

  CMD+=("${BUILD_CONTEXT}")
  echo "[docker_build.sh] Using buildx with cache: from='${BUILDX_CACHE_FROM:-}' to='${BUILDX_CACHE_TO:-}'"
  "${CMD[@]}"
else
  # Fallback to classic docker build (no explicit cache control)
  echo "[docker_build.sh] buildx not found; falling back to 'docker build' (no external cache)."
  docker build               \
    --network host           \
    -f "${DOCKERFILE_PATH}"  \
    --build-arg UID="${UID_ARG}" \
    --build-arg GID="${GID_ARG}" \
    -t "${IMAGE_TAG}" "${BUILD_CONTEXT}"
fi