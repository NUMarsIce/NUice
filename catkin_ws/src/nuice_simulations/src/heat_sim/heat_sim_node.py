#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

voltage = 0.0
drill_speed = 0.0

def speed_handler(msg):
    global drill_speed
    drill_speed=msg.data

def voltage_handler(msg):
    global voltage
    voltage=msg.data

#Calculate the energy going into heat and assume half is going into the drill.
#Calculate the temperature change assuming the drill is made of pure titanium.
def initHeatNode():
    global drill_speed
    global voltage
    pub = rospy.Publisher('heat', Float64, queue_size=10)
    rospy.init_node("heat",anonymous=True)
    rospy.Subscriber("drill_speed", Float64, speed_handler)
    rospy.Subscriber("drill_voltage", Float64, voltage_handler)
    last_cycle_drill_speed = 0.0
    temp = 25.0
    rate = rospy.Rate(10)
    while(not rospy.is_shutdown()):
        energy_change = (voltage**2)/(.0000276) - (2.56*((drill_speed)**2 - (last_cycle_drill_speed)**2))/2.0
        temp += (energy_change)/(2658.0)
        pub.publish(temp)
        last_cycle_drill_speed = drill_speed
        rate.sleep()

if __name__=='__main__':
    initHeatNode()
