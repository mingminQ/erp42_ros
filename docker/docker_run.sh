#!/bin/bash

# ------------------------------------------------------------------------------------------
# [CONTAINER NAME]
# - Optional : do not have to declare container name
# ------------------------------------------------------------------------------------------
ContainerName=$1

# ------------------------------------------------------------------------------------------
# [X11 FORWARDING PERMISSIONS]
# ------------------------------------------------------------------------------------------
echo -e "\nXhost Permissions ----------------------------------------------------\n"
xhost +local:root
xhost +local:docker

# ------------------------------------------------------------------------------------------
# [Serial devices]
# - UBLOX GPS (Moving Base)
# - UBLOX GPS (Rover)
# ------------------------------------------------------------------------------------------
echo -e "\nDevice Mounting ------------------------------------------------------\n"

DEVICES=()

# ERP42
ERP42="/dev/ttyUSB0"
if [ -e "$ERP42" ]; then
  DEVICES+=(--device "$ERP42:$ERP42")
  echo "[ERP42 Racing] ------------------ device mounted"
fi

echo -e "\n----------------------------------------------------------------------\n"

# ------------------------------------------------------------------------------------------
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
    --volume="$HOME/workspace/ros2/erp42_ros:/erp42_ros"               \
    erp42_ros:humble
