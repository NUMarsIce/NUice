from pysm import StateMachine, State, Event
from Queue import Queue
import rospy
import threading
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from std_msgs.msg import Empty
from std_msgs.msg import UInt16


class Drill(StateMachine):

    


    def __init__(self, name, drill_motion_pub, drill_rel_motion_pub, drill_speed_pub, drill_accel_pub, drill_stop_pub, drill_pub):
        super(Drill, self).__init__(name)
        self.drill_limit = True
        self.current_drill_position = 0
        self.cdp_correction = 0
        self.idle = True
        self.stopped = False
        self.drill_motion_queue = Queue(maxsize = 0)
        self.drill_goal = 0
        self.drill_motion_pub = drill_motion_pub
        self.drill_rel_motion_pub = drill_rel_motion_pub
        self.drill_speed_pub = drill_speed_pub
        self.drill_pub = drill_pub
        self.drill_stop_pub = drill_stop_pub
        self.worker_thread = threading.Thread(target=self.run)
         
        # Main states
        idle = State("idle")
        drilling = State("drilling")
        stopped = State("stopped")

        self.add_state(idle, initial=True)
        self.add_state(drilling)
        self.add_state(stopped)

        # State transitions
        self.add_transition(drilling, idle, events=['drill_idle'])
        self.add_transition(stopped, idle, events=['drill_idle'])

        self.add_transition(idle, drilling, events=['drill_drill'])
        self.add_transition(stopped, drilling, events=['drill_drill'])
        
        self.add_transition(idle, stopped, events=['drill_stop'])
        self.add_transition(drilling, stopped, events=['drill_stop'])

        # Event handlers
        idle.handlers = {'enter' : self.idleOnEnter,
                              'exit' : self.idleOnExit}
        drilling.handlers = {'enter': self.drillingOnEnter,
                                  'drill_drill' : self.drillingUpdate,
                                  'exit' : self.drillingOnExit}
        stopped.handlers = {'enter' : self.stopOnEnter,
                                 'exit' : self.stopOnExit}

        self.rate = rospy.Rate(20)
        self.drill_speed_pub.publish(600)
        drill_accel_pub.publish(400)
        self.initialize()
        print("initialized drill")
        self.worker_thread.start()

    # Subscriber callbacks
    def drillLimitCallback(self, limit_data):
        self.drill_limit = limit_data.data
    
    def drillPositionCallback(self, position_data):
        self.current_drill_position = position_data.data - self.cdp_correction

    # Action functions
    def idleOnEnter(self, state, event):
        self.idle = True
        self.drill_speed_pub.publish(600)
        print "Entered idle"

    def idleOnExit(self, state, event):
        self.idle = False
        print "Exited idle"

        
    def drillingOnEnter(self, state, event):
        self.drill_goal = -abs(event.cargo['source_event'].cargo['goal'])
        print "Entered drilling"

    def drillingUpdate(self, state, event):
        self.drill_motion_queue.put(-abs(event.cargo['goal']))
        print "Updated drilling"

    def drillingOnExit(self, state, event):
        self.drill_stop_pub.publish(Empty())
        self.drill_pub.publish(False)
        self.drill_motion_queue = Queue(maxsize=0)
        print "exited drilling"

    def stopOnEnter(self, state, event):
        self.stopped = True
        print "entered stopped"

    def stopOnExit(self, state, event):
        self.stopped = False
        print "exited stopped"

    def atZero(self):
        return (self.current_position == 0)


    # Control loop
    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()
            if self.stopped:
                continue
            if self.idle:
                if not self.drill_limit:
                    self.drill_stop_pub.publish(Empty())
                    if self.current_drill_position != 0:
                        self.cdp_correction = self.current_drill_position
                else:
                    self.drill_rel_motion_pub.publish(100)
            else:
                if (self.current_drill_position - self.drill_goal) > 3:
                    self.drill_motion_pub.publish(self.drill_goal)
                    if (self.drill_goal - self.current_drill_position) < 0:
                            self.drill_pub.publish(True)
                            self.drill_speed_pub.publish(50)
                    else:
                        self.drill_pub.publish(False)
                        self.drill_speed_pub.publish(600)
                else:
                    if not self.drill_motion_queue.empty():
                        self.drill_goal = self.drill_motion_queue.get()
                        print "changed drill goal"
                        
                    else:
                        continue

