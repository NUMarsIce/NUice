import rospy
import random

possibleLayers = [140, 50]

#Build the layers simulation, then publish material strengths. Lasts 100 seconds.
def runLayersSim():
    numLayers = randint(10,20)
    a = 1
    layers = [1000]
    while (a <= 1000):
        size = randint(a + 1,1000) - a
        strength = getNextLayerStrength()
        setNextLayer(a,size,strength,layers)
        a = a + size
    pub = rospy.Publisher('materialStrength',Float64,queue_size = 10)
    rospy.init_node('layers_node',anonymous=True)
    rate = rospy.Rate(10)
    i = 1
    while(i <= 1000 and not rospy.is_shutdown()):
        pub.publish(layers[i])
        rate.sleep()

#Get the strength of the next layer from the list of possible layer strengths.
def getNextLayerStrength():
    l = randint(0,len(possibleLayers) - 1)
    return possibleLayers[l]
    
#Build the next layer of the simulation.
def setNextLayer(a,size,strength,layers):
    for i in range(1,size):
        layers[a+i] = strength



if __name__ == '__main__':
    runLayersSim()
