#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
import random

possibleLayers = [140, 50, 80, 200, 100]
cur_position = 0.0

def position_callback(msg):
    global cur_position
    cur_position = msg.data

#Build the layers simulation, then publish material strengths. Lasts 100 seconds.
def runLayersSim():
    numLayers = random.randint(10,20)
    a = 1
    layers = []
    while (a < 1000):
        size = random.randint(a + 1,1000) - a
        strength = getNextLayerStrength()
        setNextLayer(size,strength,layers)
        a = a + size
    pub = rospy.Publisher('material_strength', Float64, queue_size = 10)
    rospy.init_node('layers_node', anonymous=True)
    rate = rospy.Rate(10)
    rospy.Subscriber("/drill_motor/cur_position", Float64, position_callback)
    while((not rospy.is_shutdown()) and cur_position < 1000):
        pub.publish(layers[int(cur_position)])
        rate.sleep()

#Get the strength of the next layer from the list of possible layer strengths.
def getNextLayerStrength():
    l = random.randint(0,len(possibleLayers) - 1)
    return possibleLayers[l]
    
#Build the next layer of the simulation.
def setNextLayer(size,strength,layers):
    for i in range(1,size):
        layers.append(strength)



if __name__ == '__main__':
    runLayersSim()
