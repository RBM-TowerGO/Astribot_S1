#!/bin/bash
set -euo pipefail
REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
WS=$(cd "$REPO/../.." && pwd)
IMAGE=towergo-offline:humble
case "${1:-help}" in
  build)
    python3 "$REPO/scripts/fetch_vendor.py"
    sudo docker build --build-arg "BASE_IMAGE=${BASE_IMAGE:-ros:humble-ros-base}" -f "$REPO/docker/Dockerfile" -t "$IMAGE" "$REPO"
    mkdir -p "$WS/data/reports"
    sudo docker image inspect "$IMAGE" > "$WS/data/reports/image-inspect.json"
    ;;
  run)
    shift
    mkdir -p "$WS/data/reports" "$WS/data/maps" "$WS/data/bags"
    IMAGE_ID=$(sudo docker image inspect --format "{{.Id}}" "$IMAGE")
    sudo docker run --rm --network none --ipc private --cap-drop ALL \
      --security-opt no-new-privileges --user "$(id -u):$(id -g)" \
      -e "TOWERGO_IMAGE_ID=$IMAGE_ID" -e HOME=/tmp -e ROS_DOMAIN_ID=125 -e ROS_LOCALHOST_ONLY=1 \
      -v "$WS/data:/data" "$IMAGE" "$@"
    ;;
  web)
    shift
    if [ "$#" -ne 1 ]; then echo 'Usage: container.sh web /data/path/to/map.yaml'; exit 2; fi
    # ROS stays container-local; only the browser HTTP port is forwarded to host loopback.
    sudo docker run --rm --init --name towergo-nav-web --network bridge --ipc private --cap-drop ALL \
      --security-opt no-new-privileges --user "$(id -u):$(id -g)" \
      -e HOME=/tmp -e ROS_DOMAIN_ID=125 -e ROS_LOCALHOST_ONLY=1 \
      -p 127.0.0.1:8766:8766 -v "$WS/data:/data:ro" \
      "$IMAGE" ros2 launch astribot_nav_bringup web_navigation.launch.py "map:=$1"
    ;;
  *) echo 'Usage: container.sh build | run <command...> | web /data/path/to/map.yaml'; exit 2;;
esac
