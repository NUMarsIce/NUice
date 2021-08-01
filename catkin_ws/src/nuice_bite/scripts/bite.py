#!/usr/bin/env python

import rospy
import pickle
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from std_msgs.msg import Int32

POSITION_STRING = 'position'
WOB_STRING = 'wob'
SPIN_SPEED_STRING = 'drill_hall'
CURRENT_STRING = 'current'

# stored infromation
feature_vector = {
    POSITION_STRING: None,
    WOB_STRING: None,
    SPIN_SPEED_STRING: None,
    CURRENT_STRING: None
}

def load_model():
    # TODO: get a correct filename
    filename = 'bite_model'
    infile = open(filename,'rb')
    model = pickle.load(infile)
    infile.close() 
    return model

def read_position(data):
    feature_vector[POSITION_STRING] = data.data

def read_wob(data):
    feature_vector[WOB_STRING] = data.data

def read_drill_hall(data):
    feature_vector[SPIN_SPEED_STRING] = data.data

def read_current(data):
    feature_vector[CURRENT_STRING] = data.

def bite():
    # load BITE from pickle
    model = load_model()

    pub = rospy.Publisher('bite_probabilities', String, queue_size=10)
    rospy.init_node('bite', anonymous=True)

    # step up feature vector subsciping information
    rospy.Subscriber("drill_loadcell/load", Float32, read_wob)
    rospy.Subscriber("drill_hall/rate", Float64, read_drill_hall)
    rospy.Subscriber("drill_stp/current_position", Int32, read_position)
    rospy.Subscriber("drill_current", Float32, read_wob)


    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        # TODO: properly format predictions output
        predictions = model.predict(feature_vector)
        pub.publish(predictions)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass


# wob drill_loadcell/load Float32 N
# spin speed drill_hall/rate Float64 RPM
# position drill_stp/current_position Int32 steps
# current made up drill_current Float32 A
