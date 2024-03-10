#!/usr/bin/env bash

set -e

BASHRC=$HOME/.bashrc
ROS_DISTRO=${ROS_DISTRO:-humble}
WORKSPACE="$HOME/ros2_ws"

function detect_and_setup_ros {
    if [[ -d /opt/ros/$ROS_DISTRO ]]; then
        source /opt/ros/$ROS_DISTRO/setup.bash
        if ! grep -q "^source /opt/ros/$ROS_DISTRO/setup.bash" $BASHRC; then
            echo -e "source /opt/ros/$ROS_DISTRO/setup.bash" >> $BASHRC
        fi
        echo "Detected and setup ROS distro '$ROS_DISTRO'."
    else
        echo "Please install ROS before running this script."
    fi
}

function install_zenoh_bridge {
    echo "deb [trusted=yes] https://download.eclipse.org/zenoh/debian-repo/ /" \
        | sudo tee -a /etc/apt/sources.list > /dev/null
    sudo apt update
    sudo apt install -y zenoh-bridge-ros2dds
}

function install_testbot_desktop_required_packages {
    install_zenoh_bridge
}

function install_testbot_desktop_package {
    mkdir -p $WORKSPACE/src
    cd $WORKSPACE/src
    git clone https://github.com/dortmans/testbot_desktop.git
    cd $WORKSPACE
    rosdep install --from-paths src -y --ignore-src
    colcon build --symlink-install
    source install/setup.bash
    if ! grep -q "^source $WORKSPACE/install/setup.bash" $BASHRC; then
        echo -e "source $WORKSPACE/install/setup.bash" >> $BASHRC
    fi
}

######################


echo "Installing 'testbot_desktop' ..."

detect_and_setup_ros

install_testbot_desktop_required_packages

install_testbot_desktop_package

echo
echo "Installation of the testbot_desktop software completed."
echo

