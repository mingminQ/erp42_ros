# erp42_msgs
ERP42 ROS2 communication interfaces

<br/>

## ControlCommand.msg
ERP42 control command includes speed, steering, brake
| Field        | Type    | Unit        | Description                                                                                                                            |
| ------------ | ------- | ----------- | -------------------------------------------------------------------------------------------------------------------------------------- |
| **speed**    | float64 | **m/s**     | The linear speed of the vehicle. Negative values are not allowed. If reverse is required, the gear must be changed in ModeCommand.srv. |
| **steering** | float64 | **rad**     | The steering angle of the vehicle. **Positive values are for the left side** and **negative values are for the right side.**           |
| **brake**    | uint8   | **0 - 150** | The braking strength of the vehicle. A value between **0 and 150** is required.                                                        |

<br/>

## Feedback.msg
Feedback information from ERP42
| Field              | Type    | Unit                          | Description                                                                                                |
| ------------------ | ------- | ----------------------------- | ---------------------------------------------------------------------------------------------------------- |
| **manual_mode**    | bool    | **Manual / Auto**             | Current control mode of the vehicle. **Ture** is **manual mode** and **false** is **auto mode**.           |
| **emergency_stop** | bool    | **On / Off**                  | Current E-Stop mode of the vehicle. **Ture** is **E-Stop On** and **false** is **E-Stop Off**.             |
| **gear**           | uint8   | **Drive / Neutral / Reverse** | Current gear of the vehicle. **Drive(0), Neutral(1), Reverse(2)**.                                         |
| **speed**          | float64 | **m/s**                       | The vehicle's current linear speed.                                                                        |
| **steering**       | float64 | **rad**                       | The vehicle's current steering angle, with **positive values for left** and **negative values for right.** |
| **brake**          | uint8   | **0 - 150**                   | The vehicle's current brake strength.                                                                      |
| **encoder_count**  | int     | **-2^31 - 2^31**              | The counter of the vehicle's wheel encoder. **PPR : 100**                                                  |
| **heartbeat**      | uint8   | **0 - 255**                   | Used to check the vehicle's communication status and activate it. It is incremented by 1 every cycle.      |

<br/>

## ModeCommand.srv
ERP42 mode command includes control mode, E-Stop, gear
| Name               | Type  | Unit                          | Description                                                                                                                                                                                       |
| ------------------ | ----- | ----------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **manual_mode**    | bool  | **Manual / Auto**             | This field enables manual mode. If **true, manual mode is enabled and auto mode is disabled**. If false, auto mode is enabled, but manual mode can be temporarily used via the manual controller. |
| **emergency_stop** | bool  | **On / Off**                  | This field enables the emergency stop. If **true, the brake value is always 150 and the control input is ignored.** If false, the emergency stop is disabled.                                     |
| **gear**           | uint8 | **Drive / Neutral / Reverse** | This field sets the vehicle's gear. It can be configured with **Drive(0), Neutral(1), or Reverse(2).** However, any value other than 0, 1, or 2 is considered neutral.                            |
