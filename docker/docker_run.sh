#!/bin/bash

# ------------------------------------------------------------------------------------------
#
# [CONTAINER NAME]
# - Optional : do not have to declare container name
# 
# [Devices]
# - ERP42 serial port mounting to docker container
#
# [DOCKER CONTAINER OPTIONS]
#
# - X11 forwarding is essential option for GUI in docker
# - No GPU environment user must add options except nvidia gpu options
#
# -it                                                      \ # terminal interaction mode
# --rm                                                     \ # remove container when exit
# --privileged                                             \ # privileged permission
# --network=host                                           \ # link with host network
# --volume="/etc/localtime:/etc/localtime:ro"              \ # time syncronization
# --env="DISPLAY=$DISPLAY"                                 \ # X11 forwarding
# --env="QT_X11_NO_MITSHM=1"                               \ # X11 forwarding
# --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"              \ # X11 forwarding
# -e NVIDIA_VISIBLE_DEVICES=all                            \ # nvidia gpu resource
# -e NVIDIA_DRIVER_CAPABILITIES=all                        \ # nvidia gpu resource
# --runtime=nvidia                                         \ # nvidia gpu resource
# --gpus all                                               \ # nvidia gpu resource
# --volume="LOCAL_FILE_PATH:DOCKER_PATH"                   \ # workspace mounting
#
# ------------------------------------------------------------------------------------------
ContainerName=$1
xhost +local:root
xhost +local:docker

echo -e "\nDevice Mounting ------------------------------------------------------\n"

# ERP42 serial port
ERP42_SERIAL_PORT="/dev/ttyUSB0"

# Declare an array variable
DEVICES=()

if [ -e "$ERP42_SERIAL_PORT" ]; then
  DEVICES+=(--device "$ERP42_SERIAL_PORT:$ERP42_SERIAL_PORT")
  echo "[ERP42] ✅"
else
  echo "[ERP42] ❌"
fi

echo -e "\n----------------------------------------------------------------------\n"

docker run                                                             \
    "${DEVICES[@]}"                                                    \
    --name=$ContainerName                                              \
    -it                                                                \
    --rm                                                               \
    --privileged                                                       \
    --network=host                                                     \
    --env ROS_DOMAIN_ID=0                                              \
    --env ROS_LOCALHOST_ONLY=0                                         \
    --env="DISPLAY=$DISPLAY"                                           \
    --env="QT_X11_NO_MITSHM=1"                                         \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"                        \
    -e NVIDIA_VISIBLE_DEVICES=all                                      \
    -e NVIDIA_DRIVER_CAPABILITIES=all                                  \
    --runtime=nvidia                                                   \
    --gpus all                                                         \
    erp42-ros:humble
