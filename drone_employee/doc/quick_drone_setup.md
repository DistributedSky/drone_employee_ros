## Quick setup

### System requrements

* MAVLink compatible flight controller (APM, Pixhawk, etc.)
* Single board computer with GNU/Linux (RaspberryPi2, etc.)
* LTE/3G dongle for the Internet

### Installation

* [Ubuntu](https://wiki.ubuntu.com/ARM/RaspberryPi)
* [ROS](http://wiki.ros.org/indigo/Installation)
* [geth](https://github.com/ethereum/go-ethereum/wiki/Installation-Instructions-for-Ubuntu)
* [aira_ros_bridge](https://github.com/aira-dao/aira_ros_bridge/tree/master/aira_ros_bridge)
* [Drone Employee Stack](https://github.com/DroneEmployee/drone_employee_ros)

### Smart contracts

Drone should have access to public(official) Ethereum network and private network **MIRA**.
Mine two [contracts](https://github.com/DroneEmployee/contracts):

* drone_employee_external.sol (official Ethereum)
* drone_employee_internal.sol (MIRA)

Constructors takes several arguments, e.g. *Air traffice controller* address(public/private),
*market* address and external drone contract also require internal contract address.

#### Tamalpais State Park

The experimental *Air traffic controller* up for *Tamalpais state park* area.

![Tamalpais area](https://raw.githubusercontent.com/DroneEmployee/drone_employee_ros/demo_march/drone_employee/doc/tamalpais_region.png)

* **MIRA** address: *0x...*
* Official Ethereum address: *0x...*
