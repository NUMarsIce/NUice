#!/usr/bin/env python
import rospy


import rospy
from std_msgs.msg import Float64

requestedSpeed = 0.0

# Collect the desired speed for the drill.
def requestHandler(request):
    global requestedSpeed
    requestedSpeed = request.data

# Collect the desired speed of the drill and calulate the current speed using various technical details of the drill hardware.
# Modify the voltage across the drill using a PID loop.
def drillInit():
    global requestedSpeed
    currentSpeed = 0.0
    currentAccel = 0.0
    torqueConstant = 10
    current = 0.0
    changeInCurrent = 0.0
    voltage = 0.0
    proportionalControl = .01
    rospy.init_node("drill_sim", anonymous=True)
    rospy.Subscriber("drill_request", Float64, requestHandler)
    pub = rospy.Publisher("drill_speed", Float64, queue_size = 10)
    vpub = rospy.Publisher("drill_voltage", Float64, queue_size = 10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        currentSpeed += currentAccel
        currentAccel = (torqueConstant*voltage)
        voltage = proportionalControl*(requestedSpeed - currentSpeed)
        pub.publish(currentSpeed)
        vpub.publish(voltage)
        rate.sleep()



if __name__ == '__main__':
    drillInit()
