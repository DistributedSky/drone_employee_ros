# Drone employee behaviour ROS package

## Draft of ROS interface

### Publish

* `/ready` :: **std_msgs/bool** - dron is ready to start
* `/wifi` :: **drone_employee/Wireless** - WiFi ESSID and password for client connection
* `/video` :: **std_msgs/string** - link to recorded video

### Subscribe

* `/target` :: **drone_employee/Target** - client position and roadmap 

