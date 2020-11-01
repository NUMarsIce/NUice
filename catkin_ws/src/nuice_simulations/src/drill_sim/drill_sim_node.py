import rospy
from std_msgs.msg import Float64

requestedSpeed = 0.0
currentSpeed = 0.0
currentAccel = 0.0
torqueConstant = ?
frictionConstant = ?
momentOfInertia = ?
current = 0.0
changeInCurrent = 0.0
voltage = 0.0
resistance = ?
efc = ?
inductance = ?
previousVoltage = 0.0
errorSum = 0.0
proportionalControl = ?
integralControl = ?
derivativeControl = ?


def requestHandler(request):
    requestedSpeed = request.data

def drillInit():
    rospy.init_node("drillSim", anonymous=True)
    rospy.Subscriber("drillRequest", Float64, requestHandler)
    pub = rospy.Publisher("drillSpeed", float, queue_size = 10)
    rospy.init_node("drillSpeed", anonymous = True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        currentSpeed += currentAccel
        current += changeInCurrent
        currentAccel = (torqueConstant*current - frictionConstant*currentSpeed)/momentOfInertia
        changeInCurrent = (voltage - resistance * current - efc * currentSpeed)/inductance
        previousVoltage = voltage
        voltage = proportionalControl*(requestedSpeed - currentSpeed) + integralControl * errorSum + derivativeControl * (voltage - previousVoltage)/(.1)
        errorSum += (requestedSpeed - currentSpeed)
        pub.publish(currentSpeed)
        rate.sleep()



if __name__ == '__main__':
    drillInit()
