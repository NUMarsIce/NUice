from pysm import StateMachine, State, Event
from Queue import Queue
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from std_msgs.msg import Empty

melt_limit = False
current_melt_position = 0

class Melt(StateMachine):

    

    def __init__(self, name, melt_motion_pub, melt_rel_motion_pub, melt_stop_pub, heater_1_pub, heater_2_pub, power_pub,
    backwash_pub, stage_1_pub, bypass_pub, air_pub, ropump_pub, mainpump_pub):
        super(StateMachine,self).__init__(name)
        self.idle = True
        self.stopped = False
        self.melt_motion_queue = Queue(maxsize = 0)
        self.melt_goal = 0
        self.melt_motion_pub = melt_motion_pub
        self.melt_rel_motion_pub = melt_rel_motion_pub
        self.melt_stop_pub = melt_stop_pub
        self.heater_1_pub = heater_1_pub
        self.heater_2_pub = heater_2_pub
        self.power_pub = power_pub
        self.backwash_pub = backwash_pub
        self.stage_1_pub = stage_1_pub
        self.bypass_pub = bypass_pub
        self.air_pub = air_pub
        self.ropump_pub = ropump_pub
        self.mainpump_pub = mainpump_pub
        self.worker_thread = threading.Thread(target=self.run)

        idle = State("idle")
        melting = State("melting")
        #rockwell = State("rockwell")
        #bowl = State("bowl")
        stopped = State("stopped")

        self.add_state(idle, initial=True)
        self.add_state(melting)
        #self.add_state(rockwell)
        #self.add_state(bowl)
        self.add_state(stopped)

        self.add_transition(idle, melting, event='melt')
        self.add_transition(stopped, melting, event='melt')
        #self.add_transition(rockwell, descending, event='descend')
        #self.add_transition(bowl, descending, event='descend')

        self.add_transition(idle, stopped, event='stop')
        self.add_transition(melting, stopped, event='stop')
        #self.add_transition(rockwell, stopped, event='stop')
        #self.add_transition(bowl, stopped, event='stop')

        self.add_transition(stopped, idle, event='idle')
        self.add_transition(melting, idle, event='idle')
        #self.add_transition(rockwell, idle, event='idle')
        #self.add_transition(bowl, idle, event='idle')

        #self.add_transition(descending, rockwell, event='rockwell')
        #self.add_transition(retracting, rockwell, event='rockwell')
        #self.add_transition(stopped, rockwell, event='rockwell')
        #self.add_transition(bowl, rockwell, event='rockwell')

        #self.add_transition(stopped, bowl, event='bowl')

        self.idle.handlers = {'enter' : self.idleOnEnter,
                              'exit' : self.idleOnExit}
        self.stopped.handlers = {'enter' : self.stopOnEnter,
                                 'exit' : self.stopOnExit}
        self.melting.handlers = {'enter' : self.meltingOnEnter,
                                 'exit' : self.meltingOnExit,
                                 'melt' : self.meltingUpdate,
                                 'heater_1' : self.heater1Update,
                                 'heater_2' : self.heater2Update,
                                 'power' : self.powerUpdate,
                                 'backwash' : self.backwashUpdate,
                                 'stage_1' : self.stage1Update,
                                 'bypass' : self.bypassUpdate,
                                 'air' : self.airUpdate,
                                 'ropump' : self.ropumpUpdate,
                                 'mainpump' : self.mainpumpUpdate}
        #self.bowl.handers = { 'enter' : self.bowlOnEnter,
         #                     'exit' : self.bowlOnExit}
        #self.rockwell.handlers = { 'enter' : self.rockwellOnEnter,
         #                          'exit' : self.rockwellOnExit}        
        
        worker_thread.start()

    def idleOnEnter(self, state, event):
        self.idle = True

    def idleOnExit(self, state, event):
        self.idle = False

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
        self.heater_1_pub.publish(False)
        self.heater_2_pub.publish(False)
        self.power_pub.publish(False)
        self.backwash_pub.publish(False)
        self.stage_1_pub.publish(False)
        self.bypass_pub.publish(False)
        self.air_pub.publish(False)
        self.ropump_pub.publish(False)
        self.mainpump_pub.publish(False)
        self.melt_motion_queue = Queue(maxsize=0)

    def run(self):
        while True:
            if self.stopped:
                continue
            if self.idle:
                if melt_limit:
                    if current_melt_position != 0:
                        self.melt_stop_pub.publish(std_msgs.msg.Empty())
                    else:
                        self.melt_rel_motion_pub.publish(-10)
            else:
                if current_melt_position != self.melt_goal:
                    self.melt_motion_pub.publish(self.melt_goal)
                else:
                    if not self.melt_motion_queue.empty():
                        self.melt_goal = self.melt_motion_queue.get()
                        if (self.melt_goal - current_melt_position) > 0:
                            self.melt_pub.publish(True)
                        else:
                            self.melt_pub.publish(False)
                    else:
                        continue

