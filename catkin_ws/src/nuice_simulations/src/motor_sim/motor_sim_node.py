#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from nuice_simulations.srv import MotorFloat64, MotorFloat64Response
import numpy as np

setpoint = 0
speed_max = .1*2 #0.1m/s
accel = 0.0001    #0.1m/s^2

def handle_setpoint(sp):
    global setpoint
    setpoint = sp.data
    return MotorFloat64Response()

def handle_relative(rel):
    global setpoint 
    setpoint += rel.data
    return MotorFloat64Response()

def handle_speed(spd):
    global speed_max 
    speed_max = spd.data/2
    return MotorFloat64Response()

def main():
    #init names
    rospy.init_node("sim_motor")
    
    pub = rospy.Publisher("cur_position", Float64, queue_size=10)
    roppub = rospy.Publisher("rate_of_penetration", Float64, queue_size=10)
    rospy.Service("setpoint", MotorFloat64, handle_setpoint)
    rospy.Service("relative_move", MotorFloat64, handle_relative)
    rospy.Service("speed", MotorFloat64, handle_speed)

    position_cur = 0
    position_prev = 0
    speed_cur = 0
    global speed_max, accel
    while not rospy.is_shutdown():
        #direction
        if position_cur > setpoint:
            accel = -abs(accel)
        elif position_cur < setpoint:
            accel = abs(accel)

        #move if not at setpoint, too lazy for deacelleration at setpoint sorry
        speed_cur = max(min(speed_cur+accel, speed_max), -speed_max)
        if np.sign(position_cur-setpoint) == np.sign(position_cur+speed_cur-setpoint):
            position_cur += speed_cur
        else:
            position_cur = setpoint
            speed_cur = 0
        
        pub.publish(position_cur)
        deltap = position_cur - position_prev
        roppub.publish(deltap)
        position_prev = position_cur
        rospy.sleep(.01) #dt of 10ms

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
