#!/usr/bin/env python

import rospy
import actionlib
from std_msgs.msg import Int32, UInt16, Bool, Empty
from nuice_msgs.msg import *



class Stepper():

    info = StepperInfo()
    info.abs_position = float("nan")
    info.abs_setpoint = float("nan")
    info.speed = float("nan")
    info.accel = float("nan")
    info.enabled = True
    info.moving = False
    info.homed = False

    home_position = 0

    def __init__(self):
        rospy.init_node('stepper', anonymous=True)
        self.stepper_name = rospy.get_param('~stepper_name', "")
        self.steps_per_unit = rospy.get_param('~steps_per_unit', 1)
        self.max_units = rospy.get_param('~max', 10000)
        self.min_units = rospy.get_param('~min', 0)
        self.reversed = rospy.get_param('~reverse', False)
        self.info.speed = rospy.get_param('~max_speed', False)
        self.info.accel = rospy.get_param('~accel', False)

        # Publish info at 10Hz
        self.info_pub = rospy.Publisher('info', StepperInfo, queue_size=10)
        rospy.Timer(rospy.Duration(0.1), self.publish_info)

        ### Wait on the driver to start publishing
        rospy.loginfo("Waiting on %s stepper..." % (self.stepper_name))
        home_position = self.info.abs_position = rospy.wait_for_message(self.stepper_name+"/current_position", Int32).data
        rospy.loginfo("%s connected!"  % (self.stepper_name))

        # DRIVERS
        rospy.Subscriber(self.stepper_name+"/current_position", Int32, self.pos_cb)

        self.abs_pub = rospy.Publisher('set_abs_pos', Int32, queue_size=10)
        self.rel_pub = rospy.Publisher('set_rel_pos', Int32, queue_size=10)
        self.speed_pub = rospy.Publisher('set_max_speed', UInt16, queue_size=10)
        self.accel_pub = rospy.Publisher('set_accel', UInt16, queue_size=10)
        self.en_pub = rospy.Publisher('set_enabled', Bool, queue_size=10)
        self.stop_pub = rospy.Publisher('quick_stop', Empty, queue_size=10)

        # Actions 
        self.goto_as = actionlib.SimpleActionServer("GoToPosition", GoToCommandAction, execute_cb=self.goto_execute_cb, auto_start = False)


        # Services


        # Publish config parameters a few time as we dont know if they actually arrive
        for _ in range(4):
            self.speed_pub(self.info.speed)
            self.accel_pub(self.info.accel)
            self.en_pub(self.info.enabled)
            rospy.sleep(0.01)



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