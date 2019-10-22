#/bin/bash

# Requirements for camera
apt install -y libraw1394-11
apt install -y libgtkmm-2.4-dev
apt install -y libglademm-2.4-dev
apt install -y libgtkglextmm-x11-1.2-dev
apt install -y libusb-1.0-0

git clone https://github.com/RhobanDeps/flycapture
cd flycapture
yes | sh install_flycapture.sh

# Requirements for GPS
apt install -y libgps-dev
apt install -y gpsd
apt install -y ros-kinetic-navigation
