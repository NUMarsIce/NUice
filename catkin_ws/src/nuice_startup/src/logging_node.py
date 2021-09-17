#!/usr/bin/env python

import sys
import rospy
from rospy.msg import AnyMsg
import os.path
from importlib import import_module

values_list = []

#Extracts data field from and AnyMsg msg
def anymsg_extract_data(anymsg):
    assert sys.version_info >= (2,7) #import_module's syntax needs 2.7
    connection_header =  anymsg._connection_header['type'].split('/')
    ros_pkg = connection_header[0] + '.msg'
    msg_type = connection_header[1]
    msg_class = getattr(import_module(ros_pkg), msg_type)
    msg = msg_class().deserialize(anymsg._buff)

    return msg.data

#Generic callback
def callback(msg, index):
    global values_list
    values_list[index] = str(anymsg_extract_data(msg))

def run():
    global values_list
    #Get the desired name of the data file and the topics to log.
    rospy.init_node("logging_node", anonymous=True)
    data_file_name = rospy.get_param("~data_file_name", "log")
    topics_list = rospy.get_param("~topics_list")
    data_rate = rospy.get_param("~hz", 10)

    #Init list to store last published values
    values_list = [None] * len(topics_list)
    
    #Subscribe to the topics in the list.
    for i in range(len(topics_list)):
        rospy.Subscriber(topics_list[i], AnyMsg, lambda msg,index=i: callback(msg, index))
    
    #Create new csv file
    if (os.path.isfile(data_file_name + ".csv")):
        i = 1
        while (os.path.isfile(data_file_name + str(i)+ ".csv")):
            i += 1
        data_file_name += str(i)
    rospy.logwarn("Logging to file %s", data_file_name + ".csv")

    #Begin logging to csv file
    with open(data_file_name + ".csv", "w") as data_file:        
        #Write topic names as header
        data_file.write("timestamp,")
        for name in topics_list:
            data_file.write(name + ',')
        data_file.write('\n')

        #Log data
        start = rospy.Time.now()
        rate = rospy.Rate(data_rate)  #Log at data_rate Hz
        while not rospy.is_shutdown():
            data_file.write(str((rospy.Time.now() - start).to_sec()) + ',')
            for i in range(len(values_list) - 1):
                data_file.write(str(values_list[i]) + ',')
            data_file.write(str(values_list[-1]) + '\n')
            data_file.flush()
            rate.sleep()

if __name__ == '__main__':
    run()




