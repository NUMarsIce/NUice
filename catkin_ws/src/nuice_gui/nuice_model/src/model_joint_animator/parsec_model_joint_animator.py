#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

joint_state = JointState()

def frame_carosel_link_cb(data):
    global joint_state
    joint_state.position[0] = data.data
def carosel_cat_link_cb(data):
    global joint_state
    joint_state.position[1] = data.data
def carosel_probe_link_cb(data):
    global joint_state
    joint_state.position[2] = data.data
def carosel_drill_link_cb(data):
    global joint_state
    joint_state.position[3] = data.data
def drill_drillbit_link_cb(data):
    global joint_state
    joint_state.position[4] = data.data
def probeshaft_probebottom_link_cb(data):
    global joint_state
    joint_state.position[5] = data.data
def probe_probeshaft_link_cb(data):
    global joint_state
    joint_state.position[6] = data.data


def main():
    global joint_state
    rospy.init_node('parsec_model_joint_animator')

    pub = rospy.Publisher('joint_states', JointState, queue_size=1)
    rospy.Subscriber("carosel_motor/cur_position", Float32, frame_carosel_link_cb)
    rospy.Subscriber("cat_axis_motor/cur_position", Float32, carosel_cat_link_cb)
    rospy.Subscriber("probe_axis_motor/cur_position", Float32, carosel_probe_link_cb)
    rospy.Subscriber("drill_axis_motor/cur_position", Float32, carosel_drill_link_cb)
    rospy.Subscriber("drill_motor/cur_position", Float32, drill_drillbit_link_cb)
    rospy.Subscriber("probe_shaft_motor/cur_position", Float32, probe_probeshaft_link_cb)
    rospy.Subscriber("probe_tilt_motor/cur_position", Float32, probeshaft_probebottom_link_cb)

    joint_state.name = ["frame_carosel_link","carosel_cat_link","carosel_probe_link","carosel_drill_link","drill_drillbit_link","probeshaft_probebottom_link","probe_probeshaft_link"]
    joint_state.position = [0]*len(joint_state.name)

    r = rospy.Rate(20) # 20hz
    while not rospy.is_shutdown():
        joint_state.header.stamp = rospy.Time.now()
        pub.publish(joint_state)
        r.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
