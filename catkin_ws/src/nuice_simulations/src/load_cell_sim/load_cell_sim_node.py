#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

drillSpeed = 1.0
materialStrength = 1.0
rop = 1.0
# A constant of proportionality that depends on mechanical details of the drill
C = 200

def speedResponse(msg):
    global drillSpeed
    drillSpeed = msg.data

def strengthResponse(msg):
    global materialStrength
    materialStrength = msg.data

def ropResponse(msg):
    global rop
    rop = msg.data

# Collect the drill speed, the current rate of penetration, and the strength of the simulated layer we are passing
# through, calculate the weight on the bit, and publish it.
def run():
    global drillSpeed
    global materialStrength
    global rop
    global C
    pub = rospy.Publisher('weight_on_bit', Float64, queue_size = 10)
    rospy.init_node('load_cell_sim_node', anonymous = True)
    rospy.Subscriber('drill_speed',Float64,speedResponse)
    rospy.Subscriber('material_strength',Float64,strengthResponse)
    rospy.Subscriber('rate_of_penetration', Float64, ropResponse)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if(drillSpeed==0):
            pub.publish(0.0)
        else:
            pub.publish((C * materialStrength * rop)/drillSpeed)
        rate.sleep()


if __name__ == '__main__':
    run()

