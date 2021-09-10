#!/usr/bin/env python

from pysm import StateMachine, State, Event
import drill_machine
import melt_machine
import threading
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from std_msgs.msg import Empty


goal = 0

class Carosel(StateMachine):

    def __init__(self, name, drill_motion_pub, drill_rel_motion_pub, drill_stop_pub, drill_pub, melt_motion_pub, melt_rel_motion_pub, melt_stop_pub, heater_1_pub, heater_2_pub, power_pub,
        backwash_pub, stage_1_pub, bypass_pub, air_pub, ropump_pub, mainpump_pub):
        super(StateMachine,self).__init__(name)
        #self.worker_thread = threading.Thread(target=self.run)
        
        # Children state machines
        self.drill = drill_machine.Drill("drilling", drill_motion_pub, drill_rel_motion_pub, drill_stop_pub, drill_pub)
        self.melt = melt_machine.Melt("melting", melt_motion_pub, melt_rel_motion_pub, melt_stop_pub, heater_1_pub, heater_2_pub, power_pub,
        backwash_pub, stage_1_pub, bypass_pub, air_pub, ropump_pub, mainpump_pub)
        
        # Main states
        #init = State("init")
        #manual = State("manual")
        #repos = State("repos")
        steady = State("steady")
        
        # Sub-States
        #self.add_state(init, initial=True)
        #self.add_state(manual)
        #self.add_state(repos)
        self.add_state(steady, initial=True)

        # Sub-state transitions
        #self.add_transition(self.init, self.manual, event='manual')
        #self.add_transition(self.repos, self.manual, event='manual')
        #self.add_transition(self.steady, self.manual, event='manual')
        #self.add_transition(self.manual, self.repos, event='reposition')
        #self.add_transition(self.steady, self.repos, event='reposition')
        #self.add_transition(self.init, self.repos, event='initialized')
        #self.add_transition(self.repos, self.steady, event='steady')

        #self.init.handlers = {'enter': self.initOnEnter}
        #self.repos.handlers = {'turn': self.turn,
         #                      'exit': self.reposExit}
        self.steady.handlers = {#'exit': self.exitSteady,
                                'drill_idle': lambda state, event: self.drill.dispatch(Event('idle')),
                                'drill_drill': lambda state, event: self.drill.dispatch(Event('drill', event.input)),
                                'drill_stop': lambda state, event: self.drill.dispatch(Event('stop')),
                                'melt_idle': lambda state, event: self.melt.dispatch(Event('idle')),
                                'melt_melt': lambda state, event: self.melt.dispatch(Event('melt', event.input)),
                                'melt_heater_1': lambda state, event: self.melt.dispatch(Event('heater_1')),
                                'melt_heater_2': lambda state, event: self.melt.dispatch(Event('heater_2')),
                                'melt_power': lambda state, event: self.melt.dispatch(Event('power')),
                                'melt_backwash': lambda state, event: self.melt.dispatch(Event('backwash')),
                                'melt_stage_1': lambda state, event: self.melt.dispatch(Event('stage_1')),
                                'melt_bypass': lambda state, event: self.melt.dispatch(Event('bypass')),
                                'melt_air': lambda state, event: self.melt.dispatch(Event('air')),
                                'melt_ropump': lambda state, event: self.melt.dispatch(Event('ropump')),
                                'melt_mainpump': lambda state, event: self.melt.dispatch(Event('mainpump')),
                                #'melt_bowl': lambda state, event: self.melt.dispatch(Event('bowl')),
                                #'melt_rockwell': lambda state, event: self.melt.dispatch(Event('rockwell')),
                                'melt_stop': lambda state, event: self.melt.dispatch(Event('stop'))}

        #worker_thread.start()

    #def turn(self, state, event):

    #def reposExit(self, state, event):

    #def exitSteady(self, state, event):
        


    #def run(self): #TODO
     #   while True:
      #      pass

def drill_limit_callback(limit_data):
    drill_limit = limit_data.data
    
def drill_position_callback(position_data):
    current_drill_position = position_data.data

def melt_limit_callback(limit_data):
    melt_limit = limit_data.data

def melt_position_callback(position_data):
    current_melt_position = position_data.data

def goal_callback(goal_data):
    goal = goal_data.data





if __name__ == '__main__':
    drill_motion_pub = rospy.Publisher("drill_stp/set_abs_pos", Int32, queue_size = 10)
    drill_rel_motion_pub = rospy.Publisher("drill_stp/set_rel_pos", Int32, queue_size = 10)
    drill_stop_pub = rospy.Publisher("drill_stp/quick_stop", Empty, queue_size = 10)
    drill_pub = rospy.Publisher("drill_relay/set_state", Bool, queue_size = 10)
    melt_motion_pub = rospy.Publisher("melt_stp/set_abs_pos", Int32, queue_size = 10)
    melt_rel_motion_pub = rospy.Publisher("melt_stp/set_rel_pos", Int32, queue_size = 10)
    melt_stop_pub = rospy.Publisher("drill_stp/quick_stop", Empty, queue_size = 10)
    heater_1_pub = rospy.Publisher("heater1_relay/set_state", Bool, queue_size = 10)
    heater_2_pub = rospy.Publisher("heater2_relay/set_state", Bool, queue_size = 10) 
    power_pub = rospy.Publisher("power_relay/set_state", Bool, queue_size = 10)
    backwash_pub = rospy.Publisher("backwash_relay/set_state", Bool, queue_size = 10)
    stage_1_pub = rospy.Publisher("stage1_relay/set_state", Bool, queue_size = 10)
    bypass_pub = rospy.Publisher("bypass_relay/set_state", Bool, queue_size = 10)
    air_pub = rospy.Publisher("air_relay/set_state", Bool, queue_size = 10)
    ropump_pub = rospy.Publisher("ropump_relay/set_state", Bool, queue_size = 10)
    mainpump_pub = rospy.Publisher("mainpump_relay/set_state", Bool, queue_size = 10)
    rospy.init_node('autonomy_core')
    rospy.Subscriber('drill_stp/current_position', Int32, drill_position_callback)
    rospy.Subscriber('drill_limit/current_state', Bool, drill_limit_callback)
    rospy.Subscriber('melt_stp/current_position', Int32, melt_position_callback)
    rospy.Subscriber('melt_limit/current_state', Bool, melt_limit_callback)
    state_machine = Carosel("carosel", drill_motion_pub, drill_rel_motion_pub, drill_stop_pub, drill_pub, melt_motion_pub, melt_rel_motion_pub,
        melt_stop_pub, heater_1_pub, heater_2_pub, power_pub,
        backwash_pub, stage_1_pub, bypass_pub, air_pub, ropump_pub, mainpump_pub)
    rospy.Subscriber('ac/goal', Int32, goal_callback)
    rospy.Subscriber('ac/events', String, lambda event_data: state_machine.dispatch(Event(event_data.data, goal)))
    rospy.spin()    




