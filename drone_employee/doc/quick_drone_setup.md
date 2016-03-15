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

Drone should have access to public(official) Ethereum network and private DAO network.
Mine two [contracts](https://github.com/DroneEmployee/contracts):

* drone_employee_external.sol (Ethereum Blockchain)
* drone_employee_internal.sol (DAO Blockchain)

Constructors takes several arguments, e.g. *Air traffice controller* address(public/private),
*market* address and external drone contract also require internal contract address.

#### Tamalpais State Park

The experimental *Air traffic controller* up for *Tamalpais state park* area.

![Tamalpais area](https://raw.githubusercontent.com/DroneEmployee/drone_employee_ros/master/drone_employee/doc/tamalpais_region.png)

**Air traffic controller:**

* **DAO Blockchain** address: *0x224946f3307201e32fb0b28abf8d2a5e76852ac1*
* **Test Ethereum Blockchain address: *0x31f0a3719e7871e513d5d395d9e3cb16c4e2d980*

**Market:**

Market placed on Test Ethereum network and has address *0x7b1c7a7c3982ac313f1829589c944749668c6a9a*.

**EtherToken:**

Ether token provide abstract tokens based on Ethereum assets for the *market*, placed on Test Ethereum network and has address *0x2eb397602ac24e5f29f8799f3b7455881b46d439*.
