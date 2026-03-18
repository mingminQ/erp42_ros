# ==================================================================================================
# Dockerfile for ROS2 Jazzy development environment
# Author: Minkyu Kil (mgkilprg@gmail.com)
# ==================================================================================================

# --------------------------------------------------------------------------------------------------
# Base image and environment variables
# --------------------------------------------------------------------------------------------------
FROM ubuntu:24.04

# Noninteractive mode
ENV DEBIAN_FRONTEND=noninteractive

# X11 MIT-SHM Disabled
ENV QT_X11_NO_MITSHM=1
ENV LIBGL_ALWAYS_INDIRECT=1

# Kakao mirror : faster in Korea
RUN sed -i \
    -e 's|http://archive.ubuntu.com/ubuntu|http://mirror.kakao.com/ubuntu|g' \
    -e 's|http://security.ubuntu.com/ubuntu|http://mirror.kakao.com/ubuntu|g' \
    /etc/apt/sources.list.d/ubuntu.sources

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
# ROS : Jazzy
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
    && echo "deb [arch=$(dpkg --print-architecture) \
    signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu \
    $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | tee /etc/apt/sources.list.d/ros2.list \
    > /dev/null

# Install ROS2 packages
RUN apt-get update && apt-get upgrade -y && apt-get install -y \
    ros-jazzy-desktop \
    ros-dev-tools \
    ros-jazzy-rmw-cyclonedds-cpp
    
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Install additional ROS2 packages
RUN apt-get update && apt-get install -y \
    ros-jazzy-rqt-tf-tree

# --------------------------------------------------------------------------------------------------
# Gazebo : Harmonic
# --------------------------------------------------------------------------------------------------
RUN curl https://packages.osrfoundation.org/gazebo.gpg \
    --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) \
    signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
    https://packages.osrfoundation.org/gazebo/ubuntu-stable \
    $(lsb_release -cs) main" \
    | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

RUN apt-get update && apt-get install -y \
    gz-harmonic 

# --------------------------------------------------------------------------------------------------
# Create user
# --------------------------------------------------------------------------------------------------

# Create a non-root user to avoid permission issues
ARG USERNAME=dev
ARG UID=1000
ARG GID=1000
RUN set -eux; \
    existing_group_name="$(getent group "${GID}" | cut -d: -f1 || true)"; \
    if [ -z "${existing_group_name}" ]; then \
        groupadd -g "${GID}" "${USERNAME}"; \
        existing_group_name="${USERNAME}"; \
    fi; \
    existing_user_name="$(getent passwd "${UID}" | cut -d: -f1 || true)"; \
    if [ -n "${existing_user_name}" ]; then \
        if [ "${existing_user_name}" != "${USERNAME}" ]; then \
            usermod -l "${USERNAME}" "${existing_user_name}"; \
            if [ -d "/home/${existing_user_name}" ]; then \
                usermod -d "/home/${USERNAME}" -m "${USERNAME}"; \
            fi; \
        fi; \
        usermod -g "${GID}" "${USERNAME}"; \
    else \
        if id -u "${USERNAME}" >/dev/null 2>&1; then \
            usermod -u "${UID}" -g "${GID}" "${USERNAME}"; \
        else \
            useradd -m -s /bin/bash -u "${UID}" -g "${GID}" "${USERNAME}"; \
        fi; \
    fi; \
    usermod -aG sudo,dialout,video,render "${USERNAME}"; \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > "/etc/sudoers.d/${USERNAME}"; \
    chmod 0440 "/etc/sudoers.d/${USERNAME}"; \
    mkdir -p "/home/${USERNAME}"; \
    chown -R "${UID}:${GID}" "/home/${USERNAME}"; \
    echo 'umask 0002' >> /etc/bash.bashrc

# --------------------------------------------------------------------------------------------------
# Build workspace
# --------------------------------------------------------------------------------------------------
WORKDIR /home/${USERNAME}/erp42_ros
COPY . .
RUN rosdep init && rosdep update
RUN rosdep install --rosdistro jazzy --from-paths src --ignore-src -r -y
RUN bash -lc "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install"

# --------------------------------------------------------------------------------------------------
# Entrypoints
# --------------------------------------------------------------------------------------------------

# Source ROS2 and workspace setup files, then launch bash
RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc
RUN echo "source /home/${USERNAME}/erp42_ros/install/setup.bash" >> /etc/bash.bashrc
USER ${USERNAME}
CMD ["bash"]