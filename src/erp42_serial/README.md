# erp42_serial
ERP42 serial communication package

<br/>

## serial_bridge
ROS2 communication interface and ERP42 PCU serial packet conversion node 

```bash
$ ros2 launch erp42_serial erp42_serial_bridge.launch.py
```

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
| Parameter Name          | Unit | Description                                                                                           |
| ----------------------- | ---- | ----------------------------------------------------------------------------------------------------- |
| **port_path**           | -    | Serial port path ( e.g. /dev/ttyUSB0 )                                                                |
| **baud_rate**           | -    | Serial communication speed. only 115200 or 9600 can be selected.                                      |
| **max_speed_mps**       | m/s  | The vehicle's maximum linear speed. When using Auto mode, the PCU limits it to 25 km/h.               |
| **max_steering_deg**    | deg  | The vehicle's maximum steering angle. Due to hardware limitations, 25 degrees or less is recommended. |
| **steering_offset_deg** | deg  | Provides an offset to the steering angle. Left is positive (+), right is negative (-).                |
