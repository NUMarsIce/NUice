#!/usr/bin/env python

import os
import rospy
import pickle
from std_msgs.msg import String, Float32, Float64, Int32
from nuice_msgs.msg import ProbabilityVector, LayerStatus


POSITION_STRING = 'position'
WOB_STRING = 'wob'
SPIN_SPEED_STRING = 'drill_hall'
CURRENT_STRING = 'current'

states = ['concrete', 'clay', 'sand', 'stone']

# stored infromation
feature_vector = {
    POSITION_STRING: 0,
    WOB_STRING: 0,
    SPIN_SPEED_STRING: 0,
    CURRENT_STRING: 0
}

def load_model():
    # TODO: get a correct filename
    filepath = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'gradient0912.sav') #logistic_model
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
    vector = ProbabilityVector()
    for state_index, state in enumerate(states):
        layer_status = LayerStatus()
        layer_status.layer_name = state
        layer_status.probability = probability_list[state_index]
        vector.probability_vector.append(layer_status)
    return vector

    
def feature_dict_to_vector():
    # TODO: update this to work for values other than wob
    return [feature_vector[CURRENT_STRING], feature_vector[SPIN_SPEED_STRING], feature_vector[WOB_STRING]]



def bite():
    # load BITE from pickle
    model = load_model()
    classes = model.classes_

    pub = rospy.Publisher('bite_probabilities', ProbabilityVector, queue_size=10)
    rospy.init_node('bite', anonymous=True)

    # step up feature vector subsciping information
    rospy.Subscriber("/drill_board/drill_loadcell/load", Float32, read_wob)
    rospy.Subscriber("/central_board/drill_stp/current_position", Int32, read_position)

    # currently commented out because initial model used for testing only has wob and position data
    rospy.Subscriber("/drill_board/drill_hall/rate", Float64, read_drill_hall)
    rospy.Subscriber("/current_board/drill/current", Float32, read_current)


    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        feature_list = feature_dict_to_vector()
        # current model requires a list of feature vectors
        probability_list = model.predict_proba([feature_list])[0]
        prediction = model.predict([feature_list])[0]

        # WILL BE USED FOR CUSTOM MESSAGING
        probability_vector = probability_list_to_probability_vector(probability_list, states)

        pub.publish(probability_vector)
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
