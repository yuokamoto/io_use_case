#!/bin/bash

set -e

# setup ros environment
source "/opt/ros/kinetic/setup.bash"
source '/ros/ws/devel/setup.bash'

exec $@

