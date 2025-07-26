#!/bin/sh

SRC_HOST="$(pwd)"/scripts
DATA_HOST="$(pwd)"/data
WS_HOST="$(pwd)"/humble_ws

DOMAIN_ID=0


docker build --build-arg DOMAIN_ID=$DOMAIN_ID -t isaac-ros:humble --file Dockerfile .

xhost + local:root

docker run \
    --name isaac-sim \
    --entrypoint bash \
    -it \
    --env-file config.env \
    --runtime nvidia \
    --gpus all \
    -e "ACCEPT_EULA=Y" \
    --rm \
    --net host \
    -v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
    -v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
    -v ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
    -v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
    -v ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
    -v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
    -v ~/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
    -v ~/docker/isaac-sim/documents:/root/Documents:rw \
    -v $SRC_HOST:/isaac-sim/project/scripts:rw \
    -v $DATA_HOST:/isaac-sim/project/data:rw \
    -v $WS_HOST:/isaac-sim/project/humble_ws:rw \
    -e "DISPLAY=:1" \
    isaac-ros:humble
