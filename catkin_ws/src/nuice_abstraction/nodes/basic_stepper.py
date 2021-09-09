#!/usr/bin/env python

import rospy
import actionlib
from std_msgs.msg import Int32, UInt16, Bool, Empty
from nuice_msgs.msg import *
from nuice_msgs.srv import *
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse


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
    abs_position_base = 0
    max_speed = 0
    home_speed = 0
    limit_state = False

    def __init__(self):
        rospy.init_node('stepper', anonymous=True)
        self.stepper_name = rospy.get_param('~stepper_name', "")
        self.steps_per_unit = rospy.get_param('~steps_per_unit', 1)
        self.max_units = rospy.get_param('~max', 10000)
        self.min_units = rospy.get_param('~min', 0)
        self.reversed = rospy.get_param('~reverse', False)
        self.info.speed = max_speed = rospy.get_param('~max_speed', 1)
        self.info.accel = rospy.get_param('~accel', 1)
        self.home_speed = rospy.get_param('~home_speed', 1)

        # Publish info at 10Hz
        self.info_pub = rospy.Publisher('info', StepperInfo, queue_size=10)
        rospy.Timer(rospy.Duration(0.1), self.publish_info)

        ### Wait on the driver to start publishing
        rospy.loginfo("Waiting on %s stepper..." % (self.stepper_name))
        # abs_position_base = rospy.wait_for_message(self.stepper_name+"/current_position", Int32).data
        rospy.loginfo("%s connected!"  % (self.stepper_name))

        # DRIVERS
        rospy.Subscriber(self.stepper_name+"/current_position", Int32, self.pos_cb)
        rospy.Subscriber(self.stepper_name+"/current_state", Bool, self.limmit_cb)

        self.abs_pub = rospy.Publisher(self.stepper_name+'/set_abs_pos', Int32, queue_size=10)
        self.rel_pub = rospy.Publisher(self.stepper_name+'/set_rel_pos', Int32, queue_size=10)
        self.speed_pub = rospy.Publisher(self.stepper_name+'/set_max_speed', UInt16, queue_size=10)
        self.accel_pub = rospy.Publisher(self.stepper_name+'/set_accel', UInt16, queue_size=10)
        self.en_pub = rospy.Publisher(self.stepper_name+'/set_enabled', Bool, queue_size=10)
        self.stop_pub = rospy.Publisher(self.stepper_name+'/quick_stop', Empty, queue_size=10)

        # Actions 
        self._goto_as = actionlib.SimpleActionServer("goto_position", GoToCommandAction, self.goto_execute_cb, False)
        self._goto_as.start()
        self._goto_feedback = GoToCommandFeedback()
        self._goto_result = GoToCommandResult()

        self._home_as = actionlib.SimpleActionServer("home", HomeCommandAction, self.home_execute_cb, False)
        self._home_as.start()
        self._home_feedback = HomeCommandFeedback()
        self._home_result = HomeCommandResult()

        # Services
        self.en_srv = rospy.Service('enable', SetBool, self.en_cb)
        self.stop_srv = rospy.Service('stop', Trigger, self.stop_cb)
        self.speed_srv = rospy.Service('set_speed', FloatCommand, self.speed_cb)

        # Publish config parameters a few time as we dont know if they actually arrive
        for _ in range(4):
            self.speed_pub.publish(self.info.speed*self.steps_per_unit)
            self.accel_pub.publish(self.info.accel*self.steps_per_unit)
            self.en_pub.publish(self.info.enabled)
            rospy.sleep(0.01)


    def home_execute_cb(self, goal):

        # Setup 
        rospy.loginfo("Homing...")    
        r = rospy.Rate(20)
        self._home_feedback.position = self.info.abs_position
        self.info.moving = True

        for _ in range(4):
            self.speed_pub.publish(UInt16(self.home_speed*self.steps_per_unit))

        # Home
        while(not self.limit_state):
            if self._goto_as.is_preempt_requested():
                rospy.loginfo("Home action canceld.")
                self.info.moving = False
                self._home_result.success = False
                self._home_as.set_preempted()
                self.stop_cb()
                return

            self.rel_pub.publish(Int32(-self.home_speed*self.steps_per_unit)) # 1 second of relative movement
            self._home_feedback.position = self.info.abs_position
            self._home_as.publish_feedback(self._home_feedback)
            r.sleep()

        # Stop
        self.stop_cb()
        rospy.sleep(1)

        # Set home
        rospy.loginfo("Homed at %f" % self.abs_position_base)    
        self.home_position = self.abs_position_base 

        # End and reset speed
        self.info.moving = False
        for _ in range(4):
            self.speed_pub.publish(UInt16(self.max_speed*self.steps_per_unit))
        
        self._home_result.success = True
        self._home_as.set_succeeded(self._home_result)


    def goto_execute_cb(self, goal):
        # Check position range
        if not (self.min_units <= goal.position <= self.max_units):
            rospy.logwarn("Position %d not in a valid range (%.3f to %.3f)." % (goal.position, self.min_units, self.max_units))
            self._goto_result.success = False
            self._goto_as.set_succeeded(self._goto_result)    
            return
                    
        # Setup 
        self.info.abs_setpoint = goal.position
        self._goto_feedback.distance = abs(self.info.abs_position-goal.position)
        rospy.loginfo("Going to %.4f." % goal.position)    
        self.info.moving = True
        
        # Request new position
        for _ in range(4):
            self.abs_pub.publish(int(goal.position*self.steps_per_unit))
            rospy.sleep(0.01)

        # Wait for move    
        while(True):
            # Handle Preemption (canceling)
            if self._goto_as.is_preempt_requested():
                rospy.loginfo("Goto action canceled.")
                self.info.moving = False
                self._goto_result.success = False
                self.stop_cb()
                self._goto_as.set_preempted()
                return

            # Break when we are at the requested position
            if(self.info.abs_position == int(goal.position*self.steps_per_unit)):
                break

            # Update Feedback
            self._goto_feedback.distance = abs(self.info.abs_position-goal.position)
            self._goto_as.publish_feedback(self._goto_feedback)
        
        self.info.moving = False
        self._goto_result.success = True
        self._goto_as.set_succeeded(self._goto_result)


    def publish_info(self, event):
        self.info_pub.publish(self.info)


    def pos_cb(self, msg):
        self.abs_position_base = float(msg.data)/self.steps_per_unit
        self.info.abs_position = self.abs_position_base - self.home_position


    def limmit_cb(self, msg):
        self.limit_state = msg.data


    def en_cb(self, req):
        self.info.enabled = req.data
        self.en_pub.publish(Bool(req.data))
        return SetBoolResponse(True, "")


    def stop_cb(self, req=None):
        self._goto_as.preempt_request = True
        for _ in range(4):
            self.stop_pub.publish(Empty())
            rospy.sleep(0.01)
        return TriggerResponse(True, "")
        

    def speed_cb(self, req):
        self.info.speed = req.data
        self.speed_pub.publish(req.data*self.steps_per_unit)
        return FloatCommandResponse(True)


if __name__ == '__main__':
    try:
        stp = Stepper()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass