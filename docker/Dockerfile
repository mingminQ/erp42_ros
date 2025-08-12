# --------------------------------------------------------------------------------
# Base Image
# --------------------------------------------------------------------------------
FROM nvidia/cuda:12.8.0-cudnn-devel-ubuntu22.04
ENV DEBIAN_FRONTEND=noninteractive

# Kakao mirror
RUN sed -i                                                                    \
    -e 's|http://archive.ubuntu.com/ubuntu|http://mirror.kakao.com/ubuntu|g'  \
    -e 's|http://security.ubuntu.com/ubuntu|http://mirror.kakao.com/ubuntu|g' \
    /etc/apt/sources.list

# --------------------------------------------------------------------------------
# Libraries : Ubuntu
# --------------------------------------------------------------------------------
RUN apt-get update && apt-get install -y \
    software-properties-common           \
    apt-transport-https                  \
    build-essential                      \
    ca-certificates                      \
    lsb-release                          \
    apt-utils                            \
    gnupg2                               \
    gedit                                \
    cmake                                \
    curl                                 \
    wget                                 \
    vim                                  \
    git         

# --------------------------------------------------------------------------------
# Ubuntu graphics
# --------------------------------------------------------------------------------
RUN apt-get update && apt-get install -y \
    libgl1-mesa-glx                      \
    libgl1-mesa-dri                      \
    libglu1-mesa                         \
    libegl1-mesa                         \
    libx11-xcb1                          \
    libxcb-xinerama0                     \
    libxcb-glx0                          \
    libxcb-icccm4                        \
    libxcb-image0                        \
    libxcb-keysyms1                      \
    libxcb-randr0                        \
    libxcb-render-util0                  \
    libxcb-shape0                        \
    libxcb-sync1                         \
    libxcb-xfixes0                       \
    libxcb-xkb1                          \
    dbus-x11                             

# X11 MIT-SHM Disabled
ENV QT_X11_NO_MITSHM=1
ENV LIBGL_ALWAYS_INDIRECT=1

# ---------------------------------------------------------------------------
# ROS : Humble
# ---------------------------------------------------------------------------
RUN apt-get install -y locales      \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale                \
    LC_ALL=en_US.UTF-8              \
    LANG=en_US.UTF-8
    
ENV LANG en_US.UTF-8

RUN ln -fs /usr/share/zoneinfo/America/Chicago /etc/localtime \
    && apt-get install -y tzdata                              \
    && dpkg-reconfigure --frontend noninteractive tzdata

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o \
    /usr/share/keyrings/ros-archive-keyring.gpg                                 \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | tee /etc/apt/sources.list.d/ros2.list \
    > /dev/null

RUN apt-get update && apt-get upgrade -y && apt-get install -y \
    ros-humble-desktop                                         \
    ros-humble-rmw-cyclonedds-cpp                              \
    python3-rosdep                                             \
    python3-colcon-common-extensions                           \
    python3-colcon-mixin                                       \
    rospack-tools
    
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# ---------------------------------------------------------------------------
# Libraries : C++
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y \
    libeigen3-dev                        
    
# ---------------------------------------------------------------------------
# Libraries : Python
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y \
    python3-vcstool                      \
    python3-pip

RUN pip3 install --upgrade \
    matplotlib             \
    numpy                  \
    pandas                 \
    pillow
    
# ---------------------------------------------------------------------------
# Libraries : ROS Packages
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# Entrypoints
# ---------------------------------------------------------------------------
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc
RUN rosdep init && rosdep update

WORKDIR /erp42_ros
CMD ["bash"]
