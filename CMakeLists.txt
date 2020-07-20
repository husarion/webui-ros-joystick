cmake_minimum_required(VERSION 2.8.3)
project(webui-ros-joystick)

find_package(catkin REQUIRED)
catkin_package( )

include_directories(
)

#############
## Install ##
#############

install(FILES
  launch/rosbot.launch
  launch/rosbot_pro.launch
  launch/rosbot_sim.launch
  launch/webui.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(PROGRAMS
  scripts/node_server.sh
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)

install(DIRECTORY
  nodejs
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
