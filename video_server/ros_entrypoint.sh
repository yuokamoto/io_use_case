#!/bin/bash

set -e

# setup ros environment
source "/opt/ros/kinetic/setup.bash"
roscore &
rosrun web_video_server web_video_server &

exec $@

