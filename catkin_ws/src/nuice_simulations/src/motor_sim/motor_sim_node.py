#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float32
import numpy as np

setpoint = 0
speed_max = .1*2 #0.1m/s
accel = 0.001    #1m/s^2

def setpoint_cb(sp):
    global setpoint
    setpoint = sp.data

def relative_cb(rel):
    global setpoint 
    setpoint += rel.data

def speed_cb(spd):
    global speed_max 
    speed_max = spd.data/2

def main():
    #init names
    rospy.init_node("sim_motor")
    
    pub = rospy.Publisher("cur_position", Float32, queue_size=1)
    rospy.Subscriber("setpoint", Float32, setpoint_cb)
    rospy.Subscriber("relative_move", Float32, relative_cb)
    rospy.Subscriber("speed", Float32, speed_cb)

    position_cur = 0;
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
        rospy.sleep(.001) #dt of 1ms

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
