# webui_ros_joystick

Simple browser-based joystick to send velocity commands to ROS device and manage software e-stop. Built as a [Node.js](https://nodejs.org/) application.

---

## ROS node API

ROS node is publishing web joystick output to `/cmd_vel` topic.
The toolbar will appear when the `e_stop` param is *true*. Node will then subscribe to `/e_stop` topic and will be able to call `e_stop_trigger` and `e_stop_reset` services.


### Publish

- `/cmd_vel` *(geometry_msgs/Twist)*

### Subscribe

- `/e_stop` *(std_msgs/Bool)*

### Service clients

- `/e_stop_trigger` *(std_srv/Trigger)*
- `/e_stop_reset` *(std_srv/Trigger)*

### Parameters

- `max_lin_vel` *(**float**, default:1.0)*
- `max_ang_vel` *(**float**, default:1.0)*
- `max_lin_accel` *(**float**, default:2.0)*
- `max_ang_accel` *(**float**, default:2.0)*
- `e_stop` *(**bool**, default:false)*
- `wait_nodes` *(**string**, default:'')*


## Docker image

[![Build/Publish Docker Image](https://github.com/husarion/webui-ros-joystick/actions/workflows/build-docker-image.yaml/badge.svg)](https://github.com/husarion/webui-ros-joystick/actions/workflows/build-docker-image.yaml)

| ROS distro | Supported architectures      |
| ---------- | ---------------------------- |
| `noetic`   | `linux/amd64`, `linux/arm64` |

Available on [Docker Hub](https://hub.docker.com/r/husarion/webui-ros-joystick/tags)

### Demo

Launch the webui joystick in the docker container. After launching, the `/cmd_vel` topic should be accessible on your host computer.
Toolbar interface can be enabled/disabled by setting the `e_stop` parameter in `docker-compose.yaml` file. A Green/Red circle around the joystick indicates if velocities will be published at `/cmd_vel` topic.

Go to the `webui-ros-joystick` folder and run:

```bash
cd webui-ros-joystick
docker compose up
```