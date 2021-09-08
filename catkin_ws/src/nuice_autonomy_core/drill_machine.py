from pysm import StateMachine, State, Event
from queue import Queue
import rospy
from std_msgs import Int32
from std_msgs import Bool
from std_msgs import Empty

global drill_limit = False
global current_drill_position = 0

class Drill(StateMachine):

    


    def __init__(self, name, drill_motion_pub, drill_rel_motion_pub, drill_stop_pub, drill_pub):
        super().__init__(name)
        #rospy.init_node("drill_machine")
        #rospy.Subscriber("drill_limit/current_state", std_msgs.msg.Bool, self.drill_limit_callback)
        #rospy.Subscriber("drill_stp/current_position", std_msgs.msg.Int32, self.drill_position_callback)
        self.idle = True
        self.stopped = False
        self.drill_motion_queue = Queue(maxsize = 0)
        self.drill_goal = 0
        self.drill_motion_pub = drill_motion_pub
        self.drill_rel_motion_pub = drill_rel_motion_pub
        self.drill_pub = drill_pub
        self.drill_stop_pub = drill_stop_pub
        #Add publisher to set 0.
        self.worker_thread = threading.Thread(target=self.run)
         

        idle = State("idle")
        drilling = State("drilling")
        stopped = State("stopped")

        self.add_state(idle, initial=True)
        self.add_state(drilling)
        self.add_state(stopped)

        self.add_transition(self.drilling, self.idle, event='idle')
        self.add_transition(self.stopped, self.idle, event='idle')

        self.add_transition(self.idle, self.drilling, event='drill')
        self.add_transition(self.stopped, self.drilling, event='drill')
        
        self.add_transition(self.idle, self.stopped, event='stop')
        self.add_transition(self.drilling, self.stopped, event='stop')

        self.idle.handlers = {'enter' : self.idleOnEnter,
                              'exit' : self.idleOnExit}
        self.drilling.handlers = {'enter': self.drillingOnEnter,
                                  'drill' : self.drillingUpdate}
        self.drilling.handlers = {'bounce' : self.bounce,
                                  'exit' : self.drillingOnExit}
        self.stopped.handlers = {'enter' : self.stopOnEnter,
                                 'exit' : self.stopOnExit}

        self.worker_thread.start()

    
    def idleOnEnter(self, state, event):
        self.idle = True

    def idleOnExit(self, state, event):
        self.idle = False
        
    def drillingOnEnter(self, state, event):
        self.drill_goal = event.input

    def drillingUpdate(self, state, event):
        self.drill_motion_queue.put(event.input)
    
    def bounce(self, state, event):
        n = self.current_drill_position
        for i in range(event.input):
            self.drill_motion_queue.put(n + 15)
            self.drill_motion_queue.put(n + 5)
            n = n + 5


    def drillingOnExit(self, state, event):
        self.drill_stop_pub.publish(std_msgs.msg.Empty())
        self.drill_pub.publish(False)
        self.drill_motion_queue = Queue(maxsize=0)

    def stopOnEnter(self, state, event):
        self.stopped = True

    def stopOnExit(self, state, event):
        self.stopped = False



    def run(self):
        while True:
            if self.stopped:
                continue
            if self.idle:
                if drill_limit:
                    if current_drill_position != 0:
                        self.drill_stop_pub.publish(std_msgs.msg.Empty())
                        #set current position to 0.
                else:
                    self.drill_rel_motion_pub.publish(-10)
            else:
                if current_drill_position != self.drill_goal:
                    self.drill_motion_pub.publish(self.drill_goal)
                else:
                    if not self.drill_motion_queue.empty():
                        self.drill_goal = self.drill_motion_queue.get()
                        if (self.drill_goal - current_drill_position) > 0:
                            self.drill_pub.publish(True)
                        else:
                            self.drill_pub.publish(False)
                    else:
                        continue

