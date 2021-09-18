#!/usr/bin/env python

from pysm import StateMachine, State, Event
import drill_machine
import melt_machine
import threading
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Bool
from std_msgs.msg import Empty
from std_msgs.msg import String
from std_msgs.msg import UInt16
from nuice_msgs.srv import FloatCommand, FloatCommandResponse




class Carosel(StateMachine):

    def __init__(self, name):
        super(Carosel,self).__init__(name)
        self.worker_thread = threading.Thread(target=self.run)
        self.goal = 0
        drill_motion_pub = rospy.Publisher("/central_board/drill_stp/set_abs_pos", Int32, queue_size = 10)
        drill_rel_motion_pub = rospy.Publisher("/central_board/drill_stp/set_rel_pos", Int32, queue_size = 10)
        drill_speed_pub = rospy.Publisher("/central_board/drill_stp/set_max_speed", UInt16, queue_size = 10)
        drill_stop_pub = rospy.Publisher("/central_board/drill_stp/quick_stop", Empty, queue_size = 10)
        drill_pub = rospy.Publisher("/central_board/drill_relay/set_state", Bool, queue_size = 10)
        melt_motion_pub = rospy.Publisher("/central_board/probe_stp/set_abs_pos", Int32, queue_size = 10)
        melt_rel_motion_pub = rospy.Publisher("/central_board/probe_stp/set_rel_pos", Int32, queue_size = 10)
        melt_speed_pub = rospy.Publisher("/central_board/probe_stp/set_max_speed", UInt16, queue_size = 10)
        melt_stop_pub = rospy.Publisher("/central_board/probe_stp/quick_stop", Empty, queue_size = 10)
        rospy.wait_for_service('set_probe1')
        probe_1_service = rospy.ServiceProxy('set_probe1', FloatCommand)
        rospy.wait_for_service('set_probe2')
        probe_2_service = rospy.ServiceProxy('set_probe2', FloatCommand)

        
        
        # Children state machines
        self.drill = drill_machine.Drill("drilling", drill_motion_pub, drill_rel_motion_pub, drill_speed_pub, drill_stop_pub, drill_pub)
        self.melt = melt_machine.Melt("melting", melt_motion_pub, melt_rel_motion_pub, melt_speed_pub, melt_stop_pub, probe_1_service, probe_2_service)

        rospy.Subscriber('/central_board/drill_stp/current_position', Int32, self.drill.drill_position_callback)
        rospy.Subscriber('/central_board/drill_limit/current_state', Bool, self.drill.drill_limit_callback)
        rospy.Subscriber('/central_board/probe_stp/current_position', Int32, self.melt.melt_position_callback)
        rospy.Subscriber('/central_board/probe_limit/current_state', Bool, self.melt.melt_limit_callback)
        rospy.Subscriber('ac/goal', Int32, self.goal_callback)
        rospy.Subscriber('ac/events', String, lambda event_data: self.dispatch(Event(event_data.data, self.goal)))
        
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
        steady.handlers = {#'exit': self.exitSteady,
                                'drill_idle': lambda state, event: self.drill.dispatch(Event('idle')),
                                'drill_drill': lambda state, event: self.drill.dispatch(Event('drill', event.input)),
                                'drill_stop': lambda state, event: self.drill.dispatch(Event('stop')),
                                'melt_idle': lambda state, event: self.melt.dispatch(Event('idle')),
                                'melt_melt': lambda state, event: self.melt.dispatch(Event('melt', event.input)),
                                'melt_stop': lambda state, event: self.melt.dispatch(Event('stop')),
                                'melt_probe_1' : lambda state, event: self.melt.dispatch(Event('probe1', event.input)),
                                'melt_probe_2' : lambda state, event: self.melt.dispatch(Event('probe2', event.input))
                                }
        self.rate = rospy.Rate(20)
        self.worker_thread.start()
    def goal_callback(self, goal_data):
        self.goal = goal_data.data

    
    

    

    #def turn(self, state, event):

    #def reposExit(self, state, event):

    #def exitSteady(self, state, event):
        


    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()







if __name__ == '__main__':
    
    rospy.init_node('autonomy_core')
    state_machine = Carosel("carosel")
    
    while not rospy.is_shutdown():
        rospy.spin()    




