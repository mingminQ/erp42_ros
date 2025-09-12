![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu_22.04-%E95420?style=flat-square&logo=ubuntu&logoColor=white)
![ROS Humble](https://img.shields.io/badge/ROS_Humble-%22314E?style=flat-square&logo=ros&logoColor=white)

![Qt](https://img.shields.io/badge/Qt-%23217346.svg?style=for-the-badge&logo=Qt&logoColor=white)
![C++](https://img.shields.io/badge/C++-%2300599C.svg?style=for-the-badge&logo=c%2B%2B&logoColor=white)

# ERP42 ROS2 packages
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

# Dependencies

**ROS2 Humble**
```
https://docs.ros.org/en/humble/Installation.html
```

**Qt5**
```bash
# Already included in ros-humble-desktop
$ sudo apt-get install -y \
  qtbase5-dev             \
  qttools5-dev-tools
```

**Gazebo**
```bash
$ sudo apt-get install -y \
  gazebo                  \
  ros-humble-gazebo-pkgs  \
  ros-humble-xacro
```
