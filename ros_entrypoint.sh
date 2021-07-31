#!/bin/bash
set -e

# setup ros environment
source "/ros_ws/devel/setup.bash"

exec "$@"