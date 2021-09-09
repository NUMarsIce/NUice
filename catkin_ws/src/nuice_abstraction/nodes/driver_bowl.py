#!/usr/bin/env python

##### TODO WORK IN PROGRESS

import rospy
from std_msgs.msg import Int32, UInt16

ROT_MIN = 0
ROT_MAX = 1600
P_MIN = 0
P_MAX = 1600

p_pos = 0
rot_pos = 0

def pitch_cb(data):
    global p_pos
    p_pos = data.data

def rot_cb(data):
    global rot_pos
    rot_pos = data.data

def main():
    global p_pos, rot_pos

    rospy.init_node("driver_bowl")

    pitch_pos_pub = rospy.Publisher("/melt_board/pitch_stp/set_abs_pos", Int32)
    pitch_speed_pub = rospy.Publisher("/melt_board/pitch_stp/set_max_speed", UInt16)
    rot_pos_pub = rospy.Publisher("/melt_board/rot_stp/set_abs_pos", Int32)
    rot_speed_pub = rospy.Publisher("/melt_board/rot_stp/set_max_speed", UInt16)

    rospy.Subscriber("/melt_board/pitch_stp/current_position", Int32, pitch_cb)
    rospy.Subscriber("/melt_board/rot_stp/current_position", Int32, rot_cb)

    ### Sketchy code start
    input("Press enter to start...")

    rot_speed_pub.publish(UInt16())



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass