#!/usr/bin/python3

import sys
import subprocess
import os

possible_arg_count = [2, 4]
if (len(sys.argv) not in possible_arg_count) or len(sys.argv)==1:
    sys.exit("""
    ###Mismatch argument count. Expected 3.###

This script set up autostart of webui ros joystick 
USAGE: 
    sudo python3 set_web_joy_upstart.py <username> <local_ip_addr> <ros_master_ip/ros_master_hostname> 
    
Example for default panther configuration: 
    sudo python3 set_web_joy_upstart.py husarion 10.15.20.2 10.15.20.3
Uninstall: 
    sudo python3 set_web_joy_upstart.py uninstall
""")


def prompt_sudo():
    ret = 0
    if os.geteuid() != 0:
        msg = "[sudo] password for %u:"
        ret = subprocess.check_call("sudo -v -p '%s'" % msg, shell=True)
    return ret


if prompt_sudo() != 0:
    sys.exit("The user wasn't authenticated as a sudoer, exiting")

if str(sys.argv[1]) == "uninstall":
    subprocess.call("rm /usr/sbin/webui_ros_joy.sh", shell=True)
    subprocess.call("rm /etc/ros/env.sh", shell=True)
    subprocess.call(
        "rm /etc/systemd/system/webui_ros_joy.service", shell=True)
    subprocess.call("systemctl disable webui_ros_joy.service", shell=True)
    sys.exit("All removed")


HOSTNAME = str(sys.argv[1])
ROS_IP = str(sys.argv[2])
ROS_MASTER_URI = str(sys.argv[3])


print("Configuration ->", "Hostname:", HOSTNAME, "ROS_IP:", ROS_IP,
      "ROS_MASTER_URI:", ROS_MASTER_URI)
subprocess.call("mkdir /usr/ros", shell=True)


#
# /etc/ros/env.sh
#

env_msg = """#!/bin/sh
export ROS_IP={rip} 
export ROS_MASTER_URI=http://{rmu}:11311
""".format(rip=ROS_IP, rmu=ROS_MASTER_URI)

subprocess.Popen(['echo "{}" > /etc/ros/env.sh'.format(env_msg)],  shell=True)



#
# /usr/sbin/webui_ros_joy.sh
#

rap_script = """#!/bin/bash
source ~/husarion_ws/devel/setup.bash
source /etc/ros/env.sh
export ROS_HOME=$(echo ~{hn})/.ros
roslaunch webui-ros-joystick webui.launch &
PID=$!
wait "$PID"
""".format(hn=HOSTNAME)

subprocess.Popen(
    ['echo "{}" > /usr/sbin/webui_ros_joy.sh'.format(rap_script)],  shell=True)


#
# webui_ros_joy service
#

rap_service = """[Unit]
After=NetworkManager.service time-sync.target 
[Service]
Type=simple
User={hn}
ExecStart=/usr/sbin/webui_ros_joy.sh
[Install]
WantedBy=multi-user.target
""".format(hn=HOSTNAME)

subprocess.Popen(
    ['echo "{}" > /etc/systemd/system/webui_ros_joy.service'.format(rap_service)],  shell=True)


subprocess.call("systemctl enable webui_ros_joy.service", shell=True)
subprocess.call("chmod +x /usr/sbin/webui_ros_joy.sh", shell=True)


print("Done!")
