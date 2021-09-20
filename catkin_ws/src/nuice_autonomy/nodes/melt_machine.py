from pysm import StateMachine, State, Event
from Queue import Queue
import rospy
import threading
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from std_msgs.msg import Empty
from std_msgs.msg import UInt16
from nuice_msgs.srv import FloatCommand, FloatCommandResponse





class Melt(StateMachine):

    def __init__(self, name, melt_motion_pub, melt_rel_motion_pub, melt_speed_pub, melt_stop_pub, probe_1_service, probe_2_service):
        super(Melt,self).__init__(name)
        self.melt_limit = True
        self.current_melt_position = 0
        self.mcp_correction = 0
        self.idle = True
        self.stopped = False
        self.melt_motion_queue = Queue(maxsize = 0)
        self.melt_goal = 0
        self.melt_motion_pub = melt_motion_pub
        self.melt_rel_motion_pub = melt_rel_motion_pub
        self.melt_speed_pub = melt_speed_pub
        self.melt_stop_pub = melt_stop_pub
        self.probe_1_service = probe_1_service
        self.probe_2_service = probe_2_service
        self.worker_thread = threading.Thread(target=self.run)

        idle = State("idle")
        melting = State("melting")
        stopped = State("stopped")

        self.add_state(idle, initial=True)
        self.add_state(melting)
        self.add_state(stopped)

        self.add_transition(idle, melting, events=['melt'])
        self.add_transition(stopped, melting, events=['melt'])

        self.add_transition(idle, stopped, events=['stop'])
        self.add_transition(melting, stopped, events=['stop'])

        self.add_transition(stopped, idle, events=['idle'])
        self.add_transition(melting, idle, events=['idle'])

        idle.handlers = {'enter' : self.idleOnEnter,
                              'exit' : self.idleOnExit}
        stopped.handlers = {'enter' : self.stopOnEnter,
                              'exit' : self.stopOnExit}
        melting.handlers = {'enter' : self.meltingOnEnter,
                                 'exit' : self.meltingOnExit,
                                 'melt' : self.meltingUpdate,
                                 'probe1' : self.probe1Update,
                                 'probe2' : self.probe2Update
                                 }      
        self.rate = rospy.Rate(20)
        self.melt_speed_pub.publish(600)
        self.worker_thread.start()

    def melt_limit_callback(self, limit_data):
        self.melt_limit = limit_data.data

    def melt_position_callback(self, position_data):
        self.current_melt_position = position_data.data - self.mcp_correction

    def idleOnEnter(self, state, event):
        self.idle = True
        self.melt_speed_pub.publish(600)

    def idleOnExit(self, state, event):
        self.idle = False
        self.melt_speed_pub.publish(100)

    def stopOnEnter(self, state, event):
        self.stopped = True

    def stopOnExit(self, state, event):
        self.stopped = False

    def meltingOnEnter(self, state, event):
        self.melt_goal = event.input

    def meltingUpdate(self, state, event):
        self.melt_goal = event.input

    def meltingOnExit(self, state, event):
        self.melt_stop_pub.publish(std_msgs.msg.Empty())
        self.melt_motion_queue = Queue(maxsize=0)

    def probe1Update(self, state, event):
        print(self.probe_1_service(event.input))

    def probe2Update(self, state, event):
        print(self.probe_2_service(event.input))

    def atZero(self):
        return (self.current_position == 0)

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()
            if self.stopped:
                continue
            if self.idle:
                if not self.melt_limit:
                    self.melt_stop_pub.publish(Empty())
                    if self.current_melt_position != 0:
                        self.cmp_correction = self.current_melt_position
                else:
                    self.melt_rel_motion_pub.publish(-100)
            else:
                if self.current_melt_position != self.melt_goal:
                    self.melt_motion_pub.publish(self.melt_goal)
                else:
                    if not self.melt_motion_queue.empty():
                        self.melt_goal = self.melt_motion_queue.get()
                        if (self.melt_goal - self.current_melt_position) > 0:
                            self.melt_pub.publish(True)
                        else:
                            self.melt_pub.publish(False)
                    else:
                        continue

