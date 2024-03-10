# testbot_desktop

Testbot management desktop.

## Installation

To install the desktop software for the remote control of the Testbot, run following command on your laptop computer:
```
wget -qO- https://raw.githubusercontent.com/dortmans/testbot_desktop/main/testbot_desktop_install.sh | bash
```
## Usage

Check the launch arguments:
```
ros2 launch testbot_desktop testbot_desktop.launch.py -s
```

To run the desktop for remote control of robot `testbot` at IP-address `192.168.2.157` :
```
ros2 launch testbot_desktop testbot_desktop.launch.py ip:=192.168.2.157
```


