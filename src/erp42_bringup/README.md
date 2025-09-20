# erp42_bringup
This package is for running **serial_bridge**, **vehicle_description**, **control_panel**, and **rviz** simultaneously.  
Note that if **vehicle_description** is not run, no Transforms will be broadcasted.

<br/>

### Parameters
Please see below for parameters related to **serial_bridge**.  
**ERP42 ROS2 serial driver :** [erp42_serial](https://github.com/mingminQ/erp42_ros/tree/humble/src/erp42_serial)

**"view_control_panel" and "view_feedback_monitor" cannot both be true. Since control_panel already includes feedback_monitor, choose only one of them.**

| Parameter Name                   | Unit | Description                                                                                                                                                                                                       |
| -------------------------------- | ---- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **erp42_bringup_parameter_file** | -    | **serial_bridge** parameter file path                                                                                                                                                                             |
| **view_control_panel**           | bool | If **True**, launches the **control_panel** and **feedback_monitor** GUIs.                                                                                                                                        |
| **view_feedback_monitor**        | bool | If **True**, launches the **feedback_monitor**.                                                                                                                                                                   |
| **launch_vehicle_description**   | bool | If **True**, **/erp42/robot_description** will be published and the Transform will be broadcast. If **False**, **/erp42/robot_description** will not be published, so the RobotModel will not be visible in Rviz. |
| **launch_rviz**                  | bool | If **True**, the Rviz file specified via **rviz_config_file** will be executed.                                                                                                                                   |
| **vehicle_description_file**     | -    | Vehicle description file path.                                                                                                                                                                                    |
| **rviz_config_file**             | -    | Rviz file path.                                                                                                                                                                                                   |