#!/usr/bin/env python


import rospy
from std_msgs.msg import Int32, UInt16

ROT_MIN = -1800
ROT_MAX = 0
ROT_SPEED = 25

P_MIN = 0
P_MAX = 2000
P_SPEED = 10

p_pos = 0
rot_pos = 0

def pitch_cb(data):
    global p_pos
    p_pos = data.data

def rot_cb(data):
    global rot_pos
    rot_pos = data.data

def main():
    global pitch_pos_pub, pitch_speed_pub, rot_pos_pub, rot_speed_pub
    rospy.init_node("driver_bowl")

    pitch_pos_pub = rospy.Publisher("/melt_board/pitch_stp/set_abs_pos", Int32, queue_size=10)
    pitch_speed_pub = rospy.Publisher("/melt_board/pitch_stp/set_max_speed", UInt16, queue_size=10)
    
    rot_pos_pub = rospy.Publisher("/melt_board/rot_stp/set_abs_pos", Int32, queue_size=10)
    rot_speed_pub = rospy.Publisher("/melt_board/rot_stp/set_max_speed", UInt16, queue_size=10)

    rospy.Subscriber("/melt_board/pitch_stp/current_position", Int32, pitch_cb)
    rospy.Subscriber("/melt_board/rot_stp/current_position", Int32, rot_cb)

    ### Sketchy code start
    raw_input("Press enter to start...")

    rot_speed_pub.publish(UInt16(ROT_SPEED))
    pitch_speed_pub.publish(UInt16(P_SPEED))

    rot_pos_pub.publish(Int32(ROT_MIN))
    pitch_pos_pub.publish(Int32(P_MIN))

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        r.sleep()

        if rot_pos == ROT_MIN:
            rot_pos_pub.publish(Int32(ROT_MAX))
        elif rot_pos == ROT_MAX:
            rot_pos_pub.publish(Int32(ROT_MIN))

        if p_pos == 0:
            pitch_pos_pub.publish(Int32(P_MAX))
        elif p_pos == P_MAX:
            print("Done with bowl!")
            break
    
    rot_pos_pub.publish(Int32(ROT_MIN))
    pitch_pos_pub.publish(Int32(P_MIN))


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        global pitch_pos_pub, pitch_speed_pub, rot_pos_pub, rot_speed_pub
        pitch_pos_pub.publish(Int32(P_MIN))
        pitch_speed_pub.publish(UInt16(P_SPEED*10))
        rot_pos_pub.publish(Int32(ROT_MIN))
        rot_speed_pub.publish(UInt16(ROT_SPEED*10))
