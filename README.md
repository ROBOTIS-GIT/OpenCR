# Overview

This repository is a fork of [ROBOTIS-GIT/OpenCR](https://github.com/ROBOTIS-GIT/OpenCR). The `read-analog-pins-tb3-ros2` branch has extended the TurtleBot3 ROS2 firmware to read data from the analog pins A0-A5 of the OpenCR board. The `master` branch is kept in sync with the original repository.

Reading analog data is a key element of the [ez-turtlebot3 project](https://github.com/ez-turtlebot3/ez-turtlebot3), which combines this OpenCR fork with an [analog-enabled fork of the turtlebot3 repo](https://github.com/ez-turtlebot3/turtlebot3/tree/publish-analog-pins) and a separate [ROS 2 analog processor package](https://github.com/ez-turtlebot3/ez_analog_processor) to process and publish the analog data.

There are also a couple of arduino sketches in [read_analog_pins_from_OpenCR](read_analog_pins_from_OpenCR) that allow the user to plug the OpenCR directly into their PC's USB port to read analog pin data, therefore bypassing the TurtleBot3's Raspberry Pi and ROS 2 altogether.

# TurtleBot3 ROS 2 Analog-Enabled Firmware Installation
1. Connect the OpenCR board to the PC via USB to micro USB.
2. Install the Arduino IDE and add the OpenCR board to the boards manager following the [Robotis E-Manual](https://emanual.robotis.com/docs/en/parts/controller/opencr10/#install-on-linux).
3. Open the IDE's Library Manager and install
  * Dynamixel2Arduino library
  * DHT sensor library by Adafruit with dependencies
4. Install the Arduino CLI
  * Make sure $HOME/.local/bin is added to your $PATH in your bashrc file:
    * `echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc`
    * `source ~/.bashrc`
  * `curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=$HOME/.local/bin sh`
  * `arduino-cli config init`
  * `arduino-cli core update-index`
  * `arduino-cli core install OpenCR:OpenCR`
5. Clone this analog-enabled OpenCR repo fork
  * `git clone https://github.com/ez-turtlebot3/OpenCR.git`
  * `cd OpenCR`
  * `git switch read-analog-pins-tb3-ros2`
6. Upload the analog-enabled firmware
  * `arduino-cli compile --upload -v -p /dev/ttyACM0 --fqbn OpenCR:OpenCR:OpenCR --libraries=$(pwd)/arduino/opencr_arduino/opencr/libraries arduino/opencr_arduino/opencr/libraries/turtlebot3_ros2/examples/turtlebot3_burger/turtlebot3_burger.ino`
7. Wait to hear the OpenCR melody, which indicates the upload was successful.
8. Disconnect the OpenCR from the remote PC.
9. Connect the OpenCR to the TurtleBot Raspberry Pi.


# License
This project is licensed under the same terms as the original OpenCR project. See the [LICENSE](LICENSE) file for details.
