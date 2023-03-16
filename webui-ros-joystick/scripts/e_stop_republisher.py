#!/usr/bin/python3

# The purpose of this node is to subscribe to the latched topic /e_stop
# and republish the messages with a constant frequency. 
# This has been created to address an error in the js ROS subscriber 
# that caused unpredictable behavior when subscribed to a latched topic.
# This is a temporal solution and this node is meant to be deleted in the future.

import rospy

from std_msgs.msg import Bool


class EStopRepublisherNode:
    def __init__(self, name: str) -> None:
        rospy.init_node(name, anonymous=False)

        self._e_stop_state = None

        # -------------------------------
        #   Subscribers
        # -------------------------------

        self._e_stop_sub = rospy.Subscriber('e_stop_in', Bool, self._e_stop_cb)

        # -------------------------------
        #   Publishers
        # -------------------------------

        self.e_stop_pub = rospy.Publisher('e_stop_out', Bool, queue_size=1)

        # -------------------------------
        #   Timers
        # -------------------------------

        # Running at 10 HZ
        self._e_stop_republisher_timer = rospy.Timer(
            rospy.Duration(0.1), self._e_stop_republisher_timer_cb
        )

        rospy.loginfo(f'[{rospy.get_name()}] Node started')

    def _e_stop_cb(self, e_stop: Bool) -> None:
        self._e_stop_state = e_stop.data

    def _e_stop_republisher_timer_cb(self, *args) -> None:
        if self._e_stop_state != None:
            self.e_stop_pub.publish(Bool(self._e_stop_state))


def main():
    e_stop_republisher_node = EStopRepublisherNode('e_stop_republisher_node')
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
