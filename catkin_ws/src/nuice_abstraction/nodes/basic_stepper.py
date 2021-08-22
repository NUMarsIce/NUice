#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from nuice_msgs.msg import StepperInfo


class Stepper():

    info = StepperInfo()
    info.abs_position = float("nan")
    info.abs_setpoint = float("nan")
    info.speed = float("nan")
    info.accel = float("nan")
    info.enabled = True
    info.moving = False
    info.homed = False

    def __init__(self):
        rospy.init_node('stepper', anonymous=True)
        self.stepper_name = rospy.get_param('~stepper_name', "")
        self.steps_per_unit = rospy.get_param('~steps_per_unit', 1)

        # Publish info at 10Hz
        self.info_pub = rospy.Publisher('info', StepperInfo, queue_size=10)
        rospy.Timer(rospy.Duration(0.1), self.publish_info)

        ### Wait on the driver to start publishing
        rospy.loginfo("Waiting on driver...")
        rospy.wait_for_message(self.stepper_name+"/current_position", Int32)
        rospy.loginfo("Driver connected!")

        # DRIVERS
        rospy.Subscriber(self.stepper_name+"/current_position", Int32, self.pos_cb)
        self._pub = rospy.Publisher('info', StepperInfo, queue_size=10)

    def publish_info(self, event):
        self.info_pub.publish(self.info)

    def pos_cb(self, msg):
        self.info.abs_position = float(msg.data)/self.steps_per_unit



if __name__ == '__main__':
    try:
        stp = Stepper()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass