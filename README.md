# testbot_desktop

Management desktop for remote monitoring and control of the [Testbot](https://github.com/dortmans/testbot) robot.

This Testbot Desktop package communicates with the Testbot using the [zenoh-bridge-ros2dds](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds).

## Installation

To install the testbot_desktop software for the remote control of the Testbot, run following command on your laptop computer:
```
wget -qO- https://raw.githubusercontent.com/dortmans/testbot_desktop/main/testbot_desktop_install.sh | bash
```
## Usage

To run the desktop for remote control of robot `testbot` at IP-address `192.168.2.30` (replace by the IP-address of your robot computer):
```
ros2 launch testbot_desktop testbot_desktop.launch.py ip:=192.168.2.30
```



