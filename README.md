![Ubuntu 24.04](https://img.shields.io/badge/Ubuntu_24.04-%23E95420?style=flat&logo=ubuntu&logoColor=white)
![Docker](https://img.shields.io/badge/Docker-%232496ED?style=flat&logo=docker&logoColor=white)
![ROS Jazzy](https://img.shields.io/badge/ROS_Jazzy-%2322314E?style=flat&logo=ros&logoColor=white)
![Qt](https://img.shields.io/badge/Qt-%2341CD52?style=flat&logo=Qt&logoColor=white)
![C++](https://img.shields.io/badge/C++-%2300599C?style=flat&logo=c%2B%2B&logoColor=white)

[![docker-build](https://img.shields.io/github/actions/workflow/status/mingminQ/erp42_ros/docker-build.yaml?branch=jazzy&label=docker-build&style=flat&logo=docker&logoColor=white&labelColor=%23181717)](https://github.com/mingminQ/erp42_ros/actions/workflows/docker-build.yaml)
[![colcon-build](https://img.shields.io/github/actions/workflow/status/mingminQ/erp42_ros/colcon-build.yaml?branch=jazzy&label=colcon-build&style=flat&logo=ros&logoColor=white&labelColor=%23181717)](https://github.com/mingminQ/erp42_ros/actions/workflows/colcon-build.yaml)

# ERP42 ROS2 Jazzy packages
If you have any suggestions or issues, please contact me using the information below.  

- **Author**  : Minkyu Kil (Sejong Univ. AIV, Graduation : 2026.02)
- **Contact** : mgkilprg@gmail.com

<br/>

# Dependencies
This package tested on ROS2 **Humble** and **Jazzy** and targets **Gazebo Harmonic**.

``` bash
$ rosdep install --rosdistro jazzy --from-paths src --ignore-src -r -y
```

<br/>

# Package overview
For more information about the package, please read the README.md at the link below.

- **ERP42 gazebo simulation :** [erp42_gazebo_sim](https://github.com/mingminQ/erp42_ros/tree/jazzy-harmonic/src/erp42_gazebo_sim)

- **ERP42 RQT Plugin :** [erp42_rqt_plugin](https://github.com/mingminQ/erp42_ros/tree/jazzy-harmonic/src/erp42_rqt_plugin)

- **ERP42 ROS2 communication interfaces :** [erp42_msgs](https://github.com/mingminQ/erp42_ros/tree/jazzy-harmonic/src/erp42_msgs)

- **ERP42 ROS2 serial driver :** [erp42_serial](https://github.com/mingminQ/erp42_ros/tree/jazzy-harmonic/src/erp42_serial)

<br/>

# Docker
For developers who want to develop in a virtual environment.

### Build
``` bash
# At your workspace directory  

$ ./docker_build.sh
```
``` bash
# If you're using a Korean network, the options below will help build your image faster.
# Otherwise, remove them.

RUN sed -i \
    -e 's|http://archive.ubuntu.com/ubuntu|http://mirror.kakao.com/ubuntu|g' \
    -e 's|http://security.ubuntu.com/ubuntu|http://mirror.kakao.com/ubuntu|g' \
    /etc/apt/sources.list.d/ubuntu.sources
```

### Launch docker container
``` bash
# At your workspace directory  

$ ./docker/docker_run.sh
```

### Conatiner Launch Script : Device Mounting
``` bash
# ERP42 serial port mounting to docker container (e.g. ERP42_SERIAL_PORT="/dev/ttyUSB0")  

ERP42_SERIAL_PORT="YOUR_DEVICE_PORT"
```

### Conatiner Launch Script : GPU Options
``` bash
# If you have an Nvidia GPU, keep the options below, otherwise remove them.

-e NVIDIA_VISIBLE_DEVICES=all
-e NVIDIA_DRIVER_CAPABILITIES=all
--gpus all
```

<br/>