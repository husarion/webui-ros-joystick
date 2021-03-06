## About

Simple browser based joystick to send velocity commands to ROS device. Built as a [Node.js](https://nodejs.org/) application.

## Installation

Install [Node.js](https://nodejs.org/):

```bash
curl -sL https://deb.nodesource.com/setup_12.x | sudo -E bash -
sudo apt install -y nodejs
```

Create workspace and clone dependency repositories, it may happen that you already have it done, in that case, skip this step:

```bash
mkdir ~/ros_workspace
mkdir ~/ros_workspace/src
cd ~/ros_workspace/src
catkin_init_workspace 
echo '. ~/ros_workspace/devel/setup.sh' >> ~/.bashrc

git clone https://github.com/husarion/rosbot_description.git
```

Clone `webui-ros-joystick` repository:

```bash
cd ~/ros_workspace/src
git clone https://github.com/husarion/webui-ros-joystick.git
```

Install dependencies:

```bash 
cd ~/ros_workspace/src/webui-ros-joystick/nodejs
npm install rosnodejs@3.0.2 socket.io@2.4.1 yargs@16.2.0 express@4.17.1
npm install
```

Build workspace:

```bash
cd ~/ros_workspace
catkin_make
. ~/ros_workspace/devel/setup.sh
```

## How to use

Panel comes with prepared launch files for `node.js` server.
Depending on your ROSbot version, you can start it with:

- for ROSbot 2.0:

    ```bash
    roslaunch webui-ros-joystick rosbot.launch
    ```

- for ROSbot 2.0 PRO:

    ```bash
    roslaunch webui-ros-joystick rosbot_pro.launch
    ```
- for Gazebo simulator:

    ```bash
    roslaunch webui-ros-joystick rosbot_sim.launch
    ```

Once all nodes are running, go to web browser and type in address bar:

```bash
ROSBOT_IP_ADDRESS:8000
```
You need to substitute phrase `ROSBOT_IP_ADDRESS` with IP address of your device.
