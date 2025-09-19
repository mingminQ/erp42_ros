#!/bin/bash

# Build the Docker image
docker build               \
  --network host           \
  -f docker/Dockerfile     \
  --build-arg UID=$(id -u) \
  --build-arg GID=$(id -g) \
  -t erp42-ros:humble .