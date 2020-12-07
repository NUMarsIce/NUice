#!/usr/bin/env python
import rospy


import rospy
from std_msgs.msg import Float64

requestedSpeed = 0.0
currentSpeed = 0.0
currentAccel = 0.0
torqueConstant = 10
current = 0.0
changeInCurrent = 0.0
voltage = 0.0
errorSum = 0.0
proportionalControl = .8
integralControl = .2

# Collect the desired speed for the drill.
def requestHandler(request):
     requestedSpeed = request.data

# Collect the desired speed of the drill and calulate the current speed using various technical details of the drill hardware.
# Modify the voltage across the drill using a PID loop.
def drillInit(requestedSpeed, currentSpeed, currentAccel, torqueConstant, current, changeInCurrent, voltage, errorSum, proportionalControl, integralControl):
    rospy.init_node("drill_sim", anonymous=True)
    rospy.Subscriber("drill_request", Float64, requestHandler)
    pub = rospy.Publisher("drill_speed", Float64, queue_size = 10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        currentSpeed += currentAccel
        currentAccel = (torqueConstant*voltage)
        voltage = proportionalControl*(requestedSpeed - currentSpeed) + integralControl * errorSum
        errorSum += (requestedSpeed - currentSpeed)
        pub.publish(currentSpeed)
        rate.sleep()



if __name__ == '__main__':
    drillInit(requestedSpeed, currentSpeed, currentAccel, torqueConstant, current, changeInCurrent, voltage, errorSum, proportionalControl, integralControl)