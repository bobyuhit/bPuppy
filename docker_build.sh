#!/bin/bash
# 构建 bPuppy Docker 镜像
# 用法: ./docker_build.sh
set -e
cd "$(dirname "$0")"
echo "Building bpuppy-builder Docker image..."
docker build -t bpuppy-builder -f Dockerfile.bpuppy .
echo "Done. Run: docker run -it --rm -v \"$(pwd):/workspace\" bpuppy-builder"
