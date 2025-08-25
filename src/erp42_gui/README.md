# erp42_gui
A GUI package that allows you to check feedback information from the ERP42 platform and issue control commands and mode commands for debugging.  

# ERROR Windows
![Cannot read erp42_serial_bridge parameters](/assets/serial_bridge_error.png)  
feedback_monitor reads parameters from erp42_serial_bridge. Therefore, if erp42_serial_bridge is not running, the following error message will appear.  

![Cannot handle /erp42/mode_command](/assets/mode_command_error.png)  
Furthermore, erp42_serial_bridge implements a service server for processing erp42_msgs/srv/ModeCommand. Therefore, if a request for erp42_msgs/srv/ModeCommand is sent without erp42_serial_bridge running, the above error message will appear.  