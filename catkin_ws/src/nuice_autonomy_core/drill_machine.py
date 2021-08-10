from pysm import StateMachine, State, Event
from queue import Queue
import rospy
from std_msgs import Int32
from std_msgs import Bool

class Drill(StateMachine):

    idle = True
    stopped = False
    drill_motion_queue = Queue(maxsize=0)
    drill_goal = 0
    drill_motion_pub = rospy.Publisher("drill_stp/set_abs_pos", std_msgs.msg.Int32)
    drill_pub = rospy.Publisher("drill_relay/set_state", std_msgs.msg.Bool)
    drill_limit = False
    current_drill_position = 0
    worker_thread = threading.Thread(target=self.run)


    def __init__(self, name):
        super().__init__(name)
        rospy.init_node("drill_machine")
        rospy.Subscriber("drill_limit/current_state", std_msgs.msg.Bool, drill_limit_callback)
        rospy.Subscriber("drill_stp/current_position", std_msgs.msg.Int32, drill_position_callback)
         

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

        self.idle.handlers = {'enter' : self.idleOnEnter}
        self.drilling.handlers = {'enter': self.drillingOnEnter}
        self.descending.handlers = {'drill' : self.drillingUpdate}
        self.stopped.handlers = {'enter' : self.stopOnEnter}
        self.stopped.handlers = {'exit' : self.stopOnExit}

        worker_thread.start()

    def drill_limit_callback(limit_data):
        drill_limit = limit_data.data
    
    def drill_position_callback(position_data):
        current_drill_position = position_data.data

    def idleOnEnter(self, state, event):
        
    def drillingOnEnter(self, state, event):
        drillingUpdate(self, state, event)

    def drillingUpdate(self, state, event):

    def stopOnEnter(self, state, event):

    def stopOnExit(self, state, event):



    def run(self): #TODO
        while True:
            if stopped:

            if idle:
            
            else:


            