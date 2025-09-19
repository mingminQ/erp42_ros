# ==================================================================================================
# Dockerfile for ROS2 Humble development environment
# Author: Minkyu Kil (mgkilprg@gmail.com)
# ==================================================================================================

# --------------------------------------------------------------------------------------------------
# Base image and environment variables
# --------------------------------------------------------------------------------------------------
FROM ubuntu:22.04

# Noninteractive mode
ENV DEBIAN_FRONTEND=noninteractive

# X11 MIT-SHM Disabled
ENV QT_X11_NO_MITSHM=1
ENV LIBGL_ALWAYS_INDIRECT=1

# Kakao mirror : faster in Korea
RUN sed -i \
    -e 's|http://archive.ubuntu.com/ubuntu|http://mirror.kakao.com/ubuntu|g' \
    -e 's|http://security.ubuntu.com/ubuntu|http://mirror.kakao.com/ubuntu|g' \
    /etc/apt/sources.list

# --------------------------------------------------------------------------------------------------
# Ubuntu packages
# --------------------------------------------------------------------------------------------------
RUN apt-get update && apt-get install -y \
    build-essential \
    gnupg2 \
    gedit \
    cmake \
    curl \
    vim \
    git

# --------------------------------------------------------------------------------------------------
# ROS : Humble
# --------------------------------------------------------------------------------------------------

# Set locale
RUN apt-get update && apt-get install -y locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Setup sources
RUN apt-get update && apt-get install -y software-properties-common \
    && add-apt-repository universe

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o \
    /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | tee /etc/apt/sources.list.d/ros2.list \
    > /dev/null

# Install ROS2 packages
RUN apt-get update && apt-get upgrade -y && apt-get install -y \
    ros-humble-desktop \
    ros-dev-tools \
    ros-humble-rmw-cyclonedds-cpp
    
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Install additional ROS2 packages
RUN apt-get update && apt-get install -y \
    ros-humble-rqt-tf-tree

# --------------------------------------------------------------------------------------------------
# Build workspace
# --------------------------------------------------------------------------------------------------
WORKDIR /erp42_ros
COPY . /erp42_ros
RUN rosdep init && rosdep update
RUN rosdep install --rosdistro humble --from-paths src --ignore-src -r -y
RUN bash -lc "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# --------------------------------------------------------------------------------------------------
# Entrypoints
# --------------------------------------------------------------------------------------------------

# Create a non-root user to avoid permission issues
ARG USERNAME=dev
ARG UID=1000
ARG GID=1000
RUN groupadd -g ${GID} ${USERNAME} \
    && useradd -m -s /bin/bash -u ${UID} -g ${GID} ${USERNAME} \
    && usermod -aG sudo,dialout,video,render ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && echo 'umask 0002' >> /etc/bash.bashrc

# Source ROS2 and workspace setup files, then launch bash
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc
RUN echo "source /erp42_ros/install/setup.bash" >> /etc/bash.bashrc
USER ${USERNAME}
CMD ["bash"]