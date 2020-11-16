#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('not_e_stop', String, queue_size=10)
    rospy.init_node('estop_sim', anonymous=True)

    while not rospy.is_shutdown():
        raw_input("press enter to send estop...")
        pub.publish("Estop by simulation node");

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
