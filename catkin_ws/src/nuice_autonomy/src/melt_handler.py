#!/usr/bin/env python
#
#   Node to handle melt state and bowl action
#


import rospy
from std_msgs.msg import String

def main():
    rospy.init_node('melt_handler', anonymous=True)
    #Define state variable
    #Define state publisher
    #Define actions
    #Init all peripherals
    #Wait on all peripherals

    while not rospy.is_shutdown():
        pass

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass