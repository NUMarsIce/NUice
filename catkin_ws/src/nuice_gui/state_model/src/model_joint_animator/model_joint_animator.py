#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

joint_state = JointState()

def frame_rack_link_cb(data):
    global joint_state
    joint_state.position[0] = data.data/1000
def drill_link_cb(data):
    global joint_state
    joint_state.position[1] = data.data/1000
def probe_link_cb(data):
    global joint_state
    joint_state.position[2] = data.data/1000

def main():
    global joint_state
    rospy.init_node('model_joint_animator')

    pub = rospy.Publisher('joint_states', JointState, queue_size=1)
    rospy.Subscriber("x_motor/cur_position", Float32, frame_rack_link_cb)
    rospy.Subscriber("drill_motor/cur_position", Float32, drill_link_cb)
    rospy.Subscriber("probe_motor/cur_position", Float32, probe_link_cb)

    joint_state.name = ["frame_rack_link","drill_link","probe_link"]
    joint_state.position = [0,0,0]

    r = rospy.Rate(20) # 10hz
    while not rospy.is_shutdown():
        joint_state.header.stamp = rospy.Time.now()
        pub.publish(joint_state);
        r.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
