#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32, UInt16, Float32
import sys

NUDGE_VAL = -50
SPEED = 100
LOAD_DELTA = 25
STEPS_PER_METER = 118110

load = 0
pos = 0

def load_cb(data):
    global load
    load = data.data

def pos_cb(data):
    global pos
    pos = data.data

def main():
    rospy.init_node("driver_ground_detection")

    rel_pos_pub = rospy.Publisher("/central_board/drill_stp/set_rel_pos", Int32, queue_size=10)
    speed_pub = rospy.Publisher("/central_board/drill_stp/set_max_speed", UInt16, queue_size=10)

    rospy.Subscriber("/central_board/drill_stp/current_position", Int32, pos_cb)
    rospy.Subscriber("/drill_board/drill_loadcell/load", Float32, load1_cb)

    speed_pub.publish(UInt16(SPEED))

    ### Sketchy code start 
    print("Waiting for load data...")
    load_base = rospy.wait_for_message("/drill_board/drill_loadcell/load", Float32) 
    print("Beginning ground detection\n")

    r = rospy.Rate(10) #10 Hz
    while not rospy.is_shutdown() and abs(load_base-load) < LOAD_DELTA:
        r.sleep()
        # move down
        rel_pos_pub.publish(Int32(NUDGE_VAL))
        print 'Pressure: %.2f | Position: %.6f\r' % (load, pos/STEPS_PER_METER),
        sys.stdout.flush()

    # stop
    rel_pos_pub.publish(Int32(0))
    
    print("\nEnd position is %.6fm (%i)" % (pos/STEPS_PER_METER, pos))


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass