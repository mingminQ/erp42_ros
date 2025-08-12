# erp42_racing_serial
ERP42 serial communication package

## serial_bridge
ROS2 communication interface and ERP42 PCU serial packet conversion node 

### Topic / Service Names
| Communication Interface | Communication Entities | Interface name | Description |
| --- | --- | --- | --- |
| Topic | Subscription | **/erp42_racing/control_command** | Control command includes speed, steering, brake |
| Topic | Publisher | **/erp42_racing/feedback** | Feedback from ERP42 |
| Servie | Server  | **/erp42_racing/mode_command** | Mode command includes control mode, E-stop, gear |

### QoS
The ModeCommand.srv service is the system default.
| QoS Policy | QoS Policy Key |
|---|---|
| History | **Keep Last** |
| Depth | **1** |
| Reliability | **Reliable** |
| Durability | **Volatile** |

### Parameters
| Parameter Name | Unit | Description |
| --- | --- | --- |
| **port_path** | - | Serial port path ( e.g. /dev/ttyUSB0 ) |
| **baud_rate** | - | Serial communication speed. only 115200 or 9600 can be selected. |
| **max_speed** | m/s | The vehicle's maximum linear speed. When using Auto mode, the PCU limits it to 25 km/h. |
| **max_steering_deg** | deg |The vehicle's maximum steering angle. Due to hardware limitations, 28 degrees or less is recommended.
| **steering_offset_deg** | deg | Provides an offset to the steering angle. Left is positive (+), right is negative (-). |