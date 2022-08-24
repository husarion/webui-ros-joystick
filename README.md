# webui_ros_joystick

Simple browser-based joystick to send velocity commands to ROS device. Built as a [Node.js](https://nodejs.org/) application.

---

## ROS node API

ROS node is publishing web joystick output to `/cmd_vel` topic.


### Publish

- `/cmd_vel` *(geometry_msgs/Twist)*


## Docker image

[![Build/Publish Docker Image](https://github.com/husarion/webui-ros-joystick/actions/workflows/build-docker-image.yaml/badge.svg)](https://github.com/husarion/webui-ros-joystick/actions/workflows/build-docker-image.yaml)

| ROS distro | Supported architectures      |
| ---------- | ---------------------------- |
| `noetic`   | `linux/amd64`, `linux/arm64` |

Available on [Docker Hub](https://hub.docker.com/r/husarion/webui-ros-joystick/tags)

### Demo

Launch the webui joystick in the docker container. After launching, the `/cmd_vel` topic should be accessible on your host computer.

Go to the `webui-ros-joystick` folder and run:

```bash
cd webui-ros-joystick
docker compose up
```