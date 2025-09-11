# erp42_gazebo
ERP42 Gazebo simulation package. Uses ERP42 platform-related properties and control plugins to define ERP42 in Gazebo.
``` bash
$ ros2 launch erp42_gazebo erp42_gazebo.launch.py
```

<div align="center">

  <img src="./assets/erp42_gazebo.png" width="50%">
  <br/>
  <figcaption>ERP42 Gazebo simulation</figcaption>

</div>

<br/><br/>

While no separate world file or sensor file is provided, all settings for using the ERP42 with Gazebo are macroized in the **.xacro** file, so you can use them as follows:
``` xml
<!-- Include files -->
<xacro:include filename="$(find erp42_description)/urdf/erp42_model.xacro"/>
<xacro:include filename="$(find erp42_gazebo)/urdf/erp42_gazebo_control.xacro"/>

<!-- ERP42 model description and gazebo control settings -->
<xacro:erp42_model/>
<xacro:erp42_gazebo_control/>
```

<br/>

The world file must contain the following items. Without these, speed and steering angle control will work fine, but the brakes will not function.
``` xml
<plugin name="gazebo_ros_properties" filename="libgazebo_ros_properties.so">
    <ros>
        <namespace>/erp42</namespace>
    </ros>
</plugin>
```

<br/>

## gazebo_bridge
This function allows you to control ERP42 in the Gazebo simulation via **erp42_msgs/msg/ControlCommand** and **erp42_msgs/srv/ModeCommand**.  
It also publishes **erp42_msgs/msg/Feedback**.

### Topic / Service Names
| Interface | Entitiy      | Type                              | Name                       | Description                                      |
| --------- | ------------ | --------------------------------- |--------------------------- | ------------------------------------------------ |
| Topic     | Subscription | **erp42_msgs/msg/ControlCommand** | **/erp42/control_command** | Control command includes speed, steering, brake  |
| Topic     | Publisher    | **erp42_msgs/msg/Feedback**       | **/erp42/feedback**        | Feedback from ERP42                              |
| Servie    | Server       | **erp42_msgs/srv/ModeCommand**    | **/erp42/mode_command**    | Mode command includes control mode, E-stop, gear |

### QoS
The ModeCommand.srv service QoS profile is the system default.
| QoS Policy  | QoS Policy Key |
| ----------- | -------------- |
| History     | **Keep Last**  |
| Depth       | **1**          |
| Reliability | **Reliable**   |
| Durability  | **Volatile**   |

### Parameters
You can set the parameters below, but they are already included in the **libgazebo_ros_ackermann_drive.so** plugin.  
Also, things like **steering_offset_deg** are not very useful because this is a simulation.
| Parameter Name          | Unit | Description                                                                                           |
| ----------------------- | ---- | ----------------------------------------------------------------------------------------------------- |
| **max_speed_mps**       | m/s  | The vehicle's maximum linear speed. When using Auto mode, the PCU limits it to 25 km/h.               |
| **max_steering_deg**    | deg  | The vehicle's maximum steering angle. Due to hardware limitations, 25 degrees or less is recommended. |
| **steering_offset_deg** | deg  | Provides an offset to the steering angle. Left is positive (+), right is negative (-).                |