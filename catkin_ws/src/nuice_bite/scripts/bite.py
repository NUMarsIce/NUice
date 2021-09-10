#!/usr/bin/env python

import os
import rospy
import pickle
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from std_msgs.msg import Int32
#from ProbabilityVector.msg import ProbabilityVector


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
    filepath = os.path.join(os.getcwd(), 'catkin_ws', 'src', 'nuice_bite', 'logistic_model.sav')
    infile = open(filepath,'rb')
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
    feature_vector[CURRENT_STRING] = data.data

def probability_list_to_probability_vector(probability_list, states):
    probability_vector = []
    for state_index in range(len(states)):
        layer_status = LayerStatus()
        layer_status.layer_name = states[state_index]
        layer_status.propability = probability_list[state_index]
        probability_vector.append(layer_status)
    
def feature_dict_to_vector():
    # TODO: update this to work for values other than wob
    return [feature_vector[POSITION_STRING], feature_vector[WOB_STRING]]


def bite():
    # load BITE from pickle
    model = load_model()
    classes = model.classes_

    # pub = rospy.Publisher('bite_probabilities', ProbabilityVector, queue_size=10)
    pub = rospy.Publisher('bite_probabilities', String, queue_size=10)
    rospy.init_node('bite', anonymous=True)

    # step up feature vector subsciping information
    rospy.Subscriber("dril/loadcell/load", Float32, read_wob)
    rospy.Subscriber("drill_stp/current_position", Int32, read_position)
    rospy.Subscriber("drill_loadcell/load", Float32, read_wob)
    rospy.Subscriber("central_board/drill_stp/current_position", Int32, read_position)

    # currently commented out because initial model used for testing only has wob and position data
    # rospy.Subscriber("drill_hall/rate", Float64, read_drill_hall)

    # rospy.Subscriber("drill_current", Float32, read_wob)


    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        feature_list = feature_dict_to_vector()
        # current model requires a list of feature vectors
        probability_list = model.predict_proba([feature_list])[0]
        prediction = model.predict(probability_list)[0]

        # WILL BE USED FOR CUSTOM MESSAGING
        # probability_vector = probability_list_to_probability_vector(probability_list, states)
        # msg = ProbabilityVector()
        # msg.probability_vector = probability_vector

        pub.publish(prediction)
        rate.sleep()

if __name__ == '__main__':
    try:
        bite()
    except rospy.ROSInterruptException:
        pass


# wob drill_loadcell/load Float32 N
# spin speed drill_hall/rate Float64 RPM
# position drill_stp/current_position Int32 steps
# current made up drill_current Float32 A
