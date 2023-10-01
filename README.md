# Notes

## Pre requisites

Add pi ruser to input group (To read /dev/input/event*): `sudo usermod -a -G input pi`

## ROS commands

First of all: `source install/setup.bash`

`ros2 launch spot-launch.xml`
`colcon build`
`ros2 pkg create --build-type ament_python --node-name spot_visu --license MIT spot_visu`
`ros2 topic list`
`ros2 topic echo /gyro`
