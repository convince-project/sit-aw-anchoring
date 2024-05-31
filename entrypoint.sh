#!/bin/bash

# source ROS
source /opt/ros/${ROS_DISTRO}/setup.bash

# Source the workspace
source /root/ros2_ws/install/setup.bash

#Execute any commands passed to the container
exec "$@"