# DroneEmployee ROS stack

This repository contains ROS packages for the **DroneEmployee** project.

## Hardware requirements

* UAV with MAVLink-compatible flight controller ([APM](http://ardupilot.com/), [PX4](http://px4.io/))
* GPS/GLONASS module
* Wireless Internet provider (LTE/3G modem)
* Linux-compatible single board computer (RaspberryPi2)

In other case you can use the *Software In The Loop* solution for testing described down.

## Installation

The **AIRA ROS Bridge** instance is needed, installation and simple usage described [there](https://github.com/aira-dao/aira_ros_bridge/tree/master/aira_ros_bridge).

Software In The Loop instance installation described [there](https://github.com/aira-dao/aira-IoT/blob/master/ROS/dron_ros_tutorial/doc/Simulation.md).

The next, create workspace and clone the repo:

    $ mkdir workspace/src -p
    $ cd workspace/src && catkin_init_workspace
    $ git clone https://github.com/DroneEmployee/drone_employee_ros.git
    $ cd ..

Install dependecies by *rosdep*:
    
    $ rosdep install --from-paths src --ignore-src --rosdistro indigo -y

Build the packages:

    $ cakin_make

### Solidity contracts

The *smart contract* should be registered before using the **Drone Employee**.
The contracts **small_atc.sol** and **drone_employee.sol** from [contracts repository](https://github.com/DroneEmployee/contracts) should be compiled by any solidity compiler, [online compier](https://chriseth.github.io/browser-solidity/) for example.
Compiled contracts passed to network by any Ethereum client, returned contract address in future used by **AIRA ROS Bridge** as argument.

## Launch

Builded workspace should be added to ROS env:

    $ source workspace/devel/setup.sh

The next you should run [GEth node](https://github.com/ethereum/go-ethereum/wiki/geth) and [AIRA ROS Bridge](https://github.com/aira-dao/aira_ros_bridge/tree/master/aira_ros_bridge):

    $ geth &
    $ cd /path/to/aira_ros_bridge 
    $ node start.js CONTRACT_ADDRESS 

### The Air Traffic Controller

Simple run the launch:

    $ roslaunch small_atc atc.launch

### The Drone Employee

#### Simulation

Run the SITL:

    $ cd /path/to/ArduCopter
    $ export PATH=$PATH:../ardupilot/Tools/autotest
    $ fg_quad_view.sh & sim_vehicle.sh --map --console

Launch the ROS node:
    
    $ roslaunch drone_employee apm_sitl.launch 

#### Hardware

Difference from upper is a using real **ArduPilot** or **PX4** hardware connected by UART or USB port.
For using serial port in launch file you should set the **fcu_url** parameter:

    $ roslaunch drone_employee apm_sitl.launch fcu_url:=/dev/ttyUSB0:115200

The argument is a serial device and baudrate.
