#!/usr/bin/env bash
set -euo pipefail
REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
WS=$(cd "$REPO/../.." && pwd)
IMAGE=towergo-real-offline:humble
case "${1:-help}" in
  build)
    DEPS="$WS/data/cache/real_dependencies"
    python3 "$REPO/scripts/fetch_real_dependencies.py" --directory "$DEPS"
    BUILD_CONTEXT=$(mktemp -d -t towergo-real-build.XXXXXX)
    trap 'rm -rf "$BUILD_CONTEXT"' EXIT
    cp -a "$REPO/docker" "$REPO/scripts" "$REPO/towergo_real" "$REPO/towergo_livox_adapter" "$BUILD_CONTEXT/"
    cp -a "$DEPS" "$BUILD_CONTEXT/real_dependencies"
    BASE_ID=$(sudo docker image inspect towergo-offline:humble --format '{{.Id}}')
    BASE_TAG="towergo-offline-base:${BASE_ID#sha256:}"
    sudo docker tag "$BASE_ID" "$BASE_TAG"
    sudo docker build --build-arg "BASE_IMAGE=$BASE_TAG" -f "$BUILD_CONTEXT/docker/Dockerfile.real" -t "$IMAGE" "$BUILD_CONTEXT"
    ;;
  run)
    shift
    IMAGE_ID=$(sudo docker image inspect "$IMAGE" --format '{{.Id}}')
    sudo docker run --rm --network none --ipc private --cap-drop ALL \
      --security-opt no-new-privileges --user "$(id -u):$(id -g)" \
      --tmpfs /real_ws/src/dependencies/fast_lio/Log:rw,mode=1777,size=64m \
      -e HOME=/tmp -e ROS_DOMAIN_ID=125 -e ROS_LOCALHOST_ONLY=1 -e "TOWERGO_IMAGE_ID=$IMAGE_ID" \
      -v "$WS/data:/data" "$IMAGE" "$@"
    ;;
  *) echo 'Usage: real_container.sh build | run <command...>'; exit 2;;
esac
