#!/usr/bin/env python

from yaml.events import SequenceStartEvent
import rospy
from nuice_msgs.srv import FloatCommand,FloatCommandResponse
from std_msgs.msg import Int32, Bool


setpoint1 = 0
setpoint2 = 0

temperature1 = 1000
temperature2 = 1000

def handle_command1(req):
    global setpoint1 
    setpoint1 = req.data
    return FloatCommandResponse(True)

def handle_command2(req):
    global setpoint2
    setpoint2 = req.data
    return FloatCommandResponse(True)

def t1_cb(msg):
    global temperature1
    temperature1 = msg.data

def t2_cb(msg):
    global temperature2
    temperature2 = msg.data

def main():
    global setpoint1, setpoint2, temperature1, temperature2

    rospy.init_node("probe_control", anonymous=True)

    srv1 = rospy.Service('set_probe1', FloatCommand, handle_command1)
    srv2 = rospy.Service('set_probe2', FloatCommand, handle_command2)
  
    h1_pub = rospy.Publisher('/central_board/heater1_relay/set_state', Bool, queue_size=10)
    h2_pub = rospy.Publisher('/central_board/heater2_relay/set_state', Bool, queue_size=10)

    rospy.Subscriber("/melt_board/probe_therm1/value", Int32, t1_cb)
    rospy.Subscriber("/melt_board/probe_therm2/value", Int32, t2_cb)

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        rate.sleep()
        h1_pub.publish(Bool(temperature1 < setpoint1))
        h2_pub.publish(Bool(temperature2 < setpoint2))
        # rospy.logwarn("t1:%f, s1:%f" % (temperature1, setpoint1))

    # Force off heaters a few times (I dont trust our drivers)
    for _ in range(4):
        h1_pub.publish(Bool(False))
        h2_pub.publish(Bool(False))

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass