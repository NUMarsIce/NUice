#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32, UInt16, Float32
import sys

l1 = 0
l2 = 0
lavg = 0
pos = 0

NUDGE_VAL = 50
SPEED = 100

def load1_cb(data):
    global l1, l2, lavg
    l1 = data.data
    lavg = (l1+l2)/2

def load2_cb(data):
    global l1, l2, lavg
    l2 = data.data
    lavg = (l1+l2)/2

def pos_cb(data):
    global pos
    pos = data.data

def main():
    rospy.init_node("driver_melt")

    rel_pos_pub = rospy.Publisher("/central_board/probe_stp/set_rel_pos", Int32, queue_size=10)
    speed_pub = rospy.Publisher("/central_board/probe_stp/set_max_speed", UInt16, queue_size=10)

    rospy.Subscriber("/central_board/probe_stp/current_position", Int32, pos_cb)
    rospy.Subscriber("/melt_board/loadcell1/load", Float32, load1_cb)
    rospy.Subscriber("/melt_board/loadcell2/load", Float32, load2_cb)

    speed_pub.publish(UInt16(SPEED))

    ### Sketchy code start
    try:
        max_press = int(raw_input("Enter pressure [100]:"))
    except ValueError:
        max_press = 100

    if not (0 < max_press <= 200):
        print("Out of range, using default pressure")
        max_press = 100
    
    r = rospy.Rate(10) #10 Hz
    while not rospy.is_shutdown():
        r.sleep()

        if(lavg < max_press):
            rel_pos_pub.publish(Int32(NUDGE_VAL))
        
        print 'Pressure: %.2f\r' % lavg,
        sys.stdout.flush()

        



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass