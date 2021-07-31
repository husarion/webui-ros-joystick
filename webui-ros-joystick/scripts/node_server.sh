#!/bin/bash
echo $(pwd)
PACKAGE_DIR=$(rospack find webui-ros-joystick)
cd $PACKAGE_DIR/nodejs
node main.js $@