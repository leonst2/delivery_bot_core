#!/usr/bin/env bash
# Sources ROS 2 Jazzy and the ros2_rust overlay, then runs the command.
set -e

source /opt/ros/jazzy/setup.bash
source /opt/ros2_rust/install/setup.bash

exec "$@"
