#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CONTAINER=gazebo-rgb-sorting-dev
IMAGE=gazebo-rgb-sorting:jazzy-harmonic
if ! docker info >/dev/null 2>&1; then
  echo 'Docker engine is unavailable. Start Docker Desktop, then rerun setup.' >&2;exit 1
fi
if docker container inspect "$CONTAINER" >/dev/null 2>&1; then
  mount_source="$(docker inspect --format '{{range .Mounts}}{{if eq .Destination "/project"}}{{.Source}}{{end}}{{end}}' "$CONTAINER")"
  if [[ "$mount_source" != "$ROOT" ]]; then echo 'Existing project container mounts a different directory; refusing to alter it.' >&2;exit 1;fi
  docker start "$CONTAINER" >/dev/null
else
  if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
    docker build --platform linux/arm64 -t "$IMAGE" -f "$ROOT/docker/Dockerfile" "$ROOT"
  fi
  docker run -d --init --name "$CONTAINER" --memory 3g --cpus 4 --shm-size 256m \
    -e LIBGL_ALWAYS_SOFTWARE=1 -e GZ_PARTITION=gazebo_rgb_sorting \
    -p 127.0.0.1:6082:6080 -v "$ROOT:/project" "$IMAGE" >/dev/null
fi
docker exec "$CONTAINER" bash -lc 'python3 -c "import cv2,numpy;print(\"OpenCV\",cv2.__version__,\"NumPy\",numpy.__version__)"; cmake -S /project/plugins -B /project/build && cmake --build /project/build -j2'
echo 'Project runtime is ready. Use scripts/run_demo.sh for the live experiment.'
