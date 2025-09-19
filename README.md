![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu_22.04-%23E95420?style=flat&logo=ubuntu&logoColor=white)
![ROS Humble](https://img.shields.io/badge/ROS_Humble-%2322314E?style=flat&logo=ros&logoColor=white)
![Qt](https://img.shields.io/badge/Qt-%2341CD52?style=flat&logo=Qt&logoColor=white)
![C++](https://img.shields.io/badge/C++-%2300599C?style=flat&logo=c%2B%2B&logoColor=white)


# ERP42 ROS2 Humble packages
If you have any suggestions or issues, please contact me using the information below.  

- **Author**  : Minkyu Kil (Sejong Univ. AIV, Graduation : 2026.02)
- **Contact** : mgkilprg@gmail.com

<br/>

# Package overview
For more information about the package, please read the README.md at the link below.

- **ERP42 bringup :** [erp42_bringup](https://github.com/mingminQ/erp42_ros/tree/humble/src/erp42_bringup)

- **ERP42 description files :** [erp42_description](https://github.com/mingminQ/erp42_ros/tree/humble/src/erp42_description)

- **ERP42 gazebo simulation :** [erp42_gazebo](https://github.com/mingminQ/erp42_ros/tree/humble/src/erp42_gazebo)

- **ERP42 GUI :** [erp42_gui](https://github.com/mingminQ/erp42_ros/tree/humble/src/erp42_gui)

- **ERP42 ROS2 communication interfaces :** [erp42_msgs](https://github.com/mingminQ/erp42_ros/tree/humble/src/erp42_msgs)

- **ERP42 ROS2 serial driver :** [erp42_serial](https://github.com/mingminQ/erp42_ros/tree/humble/src/erp42_serial)

<br/>

# Docker
For developers who want to develop in a virtual environment.

### Build
``` bash
# At your workspace directory  

$ docker build -f docker/Dockerfile -t erp42-ros:humble .
```
``` bash
# If you're using a Korean network, the options below will help build your image faster.
# Otherwise, remove them.

RUN sed -i \
    -e 's|http://archive.ubuntu.com/ubuntu|http://mirror.kakao.com/ubuntu|g' \
    -e 's|http://security.ubuntu.com/ubuntu|http://mirror.kakao.com/ubuntu|g' \
    /etc/apt/sources.list
```

### Launch docker container
``` bash
# At your workspace directory  

$ . docker/docker_run.sh
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
--runtime=nvidia 
--gpus all
```

<br/>

# Dependencies

### Automatic Installation
``` bash
$ rosdep install --rosdistro humble --from-paths src --ignore-src -r -y
```

### Manual Installation
``` bash
$ sudo apt-get install -y    \
  qtbase5-dev                \
  ros-humble-xacro           \
  gazebo                     \
  ros-humble-gazebo-ros      \
  ros-humble-gazebo-ros-pkgs
```